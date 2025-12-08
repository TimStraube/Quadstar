#include "uart_forward.h"
#include <string.h>
#include <stdlib.h>

// CRC16-CCITT (0x1021), init 0xFFFF
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc <<= 1;
    }
  }
  return crc;
}

// small blocking helper to read exact number of bytes with HAL_UART_Receive
static HAL_StatusTypeDef uart_read_exact(UART_HandleTypeDef *huart, uint8_t *buf, size_t len, uint32_t timeout_ms) {
  if (len == 0) return HAL_OK;
  return HAL_UART_Receive(huart, buf, len, timeout_ms);
}

int UARTF_ReceiveOneFrame(UART_HandleTypeDef *huart, QuadcomPacket *out_pkt) {
  uint8_t b;
  HAL_StatusTypeDef res;

  // Read first header byte (blocking short timeout)
  res = uart_read_exact(huart, &b, 1, 200);
  if (res != HAL_OK) return 0; // no data
  if (b != 0xAA) {
    // try resync: return and let caller call again
    return 0;
  }

  // read second header
  res = uart_read_exact(huart, &b, 1, 100);
  if (res != HAL_OK) return 0;
  if (b != 0x55) {
    return 0;
  }

  // read length (2 bytes little-endian)
  uint8_t lenbuf[2];
  res = uart_read_exact(huart, lenbuf, 2, 100);
  if (res != HAL_OK) return -1;
  uint16_t payload_len = (uint16_t)lenbuf[0] | ((uint16_t)lenbuf[1] << 8);

  // Accept either a full QuadcomPacket or a 1-byte PauseMessage
  if (payload_len == sizeof(QuadcomPacket)) {
    uint8_t payload[sizeof(QuadcomPacket)];
    res = uart_read_exact(huart, payload, payload_len, 500);
    if (res != HAL_OK) return -1;

    uint8_t crcbuf[2];
    res = uart_read_exact(huart, crcbuf, 2, 100);
    if (res != HAL_OK) return -1;
    uint16_t crc_recv = (uint16_t)crcbuf[0] | ((uint16_t)crcbuf[1] << 8);

    uint16_t crc_calc = crc16_ccitt(payload, payload_len);
    if (crc_calc != crc_recv) {
      return -1;
    }

    memcpy(out_pkt, payload, sizeof(QuadcomPacket));
    out_pkt->statusMessage[sizeof(out_pkt->statusMessage)-1] = '\0';
    return 1;
  } else if (payload_len == sizeof(uint8_t)) {
    // PauseMessage (1 byte)
    uint8_t p;
    res = uart_read_exact(huart, &p, 1, 200);
    if (res != HAL_OK) return -1;
    uint8_t crcbuf[2];
    res = uart_read_exact(huart, crcbuf, 2, 100);
    if (res != HAL_OK) return -1;
    uint16_t crc_recv = (uint16_t)crcbuf[0] | ((uint16_t)crcbuf[1] << 8);
    uint16_t crc_calc = crc16_ccitt(&p, 1);
    if (crc_calc != crc_recv) return -1;
    // call weak pause handler
    UARTF_ProcessPause(p);
    return 2;
  } else {
    // unknown length -> discard payload_len + CRC to resync (best effort)
    uint32_t toDiscard = (uint32_t)payload_len + 2;
    uint8_t tmp[32];
    while (toDiscard) {
      uint32_t chunk = (toDiscard > sizeof(tmp)) ? sizeof(tmp) : toDiscard;
      if (uart_read_exact(huart, tmp, chunk, 200) != HAL_OK) break;
      toDiscard -= chunk;
    }
    return -1;
  }
}

// weak symbol so user can override in their code, or link-time provide custom
__weak void UARTF_ProcessPacket(const QuadcomPacket *pkt) {
  // Default implementation: toggle LED / simple handling
  // User can implement a stronger symbol in their main.c or another file.
  (void)pkt;
}

void UARTF_Init(void) {
  // currently nothing to init for blocking mode; place-holder for DMA init
}

__weak void UARTF_ProcessPause(uint8_t pauseValue) {
  // Default pause handler: no-op. User may override in main.c.
  (void)pauseValue;
}
