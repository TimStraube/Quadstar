/* uart_forward.h
 * Simple UART frame receiver for QuadcomPacket
 */
#ifndef UART_FORWARD_H
#define UART_FORWARD_H

#include <stdint.h>
#include "main.h"

#pragma pack(push,1)
typedef struct {
  int8_t state;
  int16_t setpoint_roll;
  int16_t setpoint_pitch;
  int16_t setpoint_yaw;
  int16_t thrust;
  int16_t setpoint_delta_north;
  int16_t setpoint_delta_east;
  int16_t setpoint_delta_down;
  uint8_t battery;
  char statusMessage[64];
} QuadcomPacket;
#pragma pack(pop)

// Initialize module if needed (optional)
void UARTF_Init(void);

// Try to receive one frame from uart handle huart (blocking with timeouts)
// Returns: 1 on success (out_pkt filled), 0 on no data, -1 on error (CRC/len)
int UARTF_ReceiveOneFrame(UART_HandleTypeDef *huart, QuadcomPacket *out_pkt);

// Process a received packet (user callback stub you can implement)
void UARTF_ProcessPacket(const QuadcomPacket *pkt);

#endif // UART_FORWARD_H
