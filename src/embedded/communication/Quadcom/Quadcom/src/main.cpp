#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define LED_GPIO 2
#define BUTTON_GPIO 18
#define PWM_PIN 21
// Hardware UART (to main compute unit / STM32)
#define UART_TX_PIN 17
#define UART_RX_PIN 16

// Additional PWM outputs for roll and pitch setpoints (separate wires to STM32)
#define ROLL_PWM_PIN 22
#define PITCH_PWM_PIN 23

const unsigned long blinkInterval = 500;
unsigned long lastBlink = 0;
bool ledState = false;
// Error blink configuration
const unsigned long errorBlinkInterval = 150; // faster blink when error
bool errorState = false;
// Active state: when true the LED should be kept permanently on
bool idleState = false;

// Dummy-Struktur für das Pause-Signal
typedef struct __attribute__((packed)) {
  uint8_t pause; // Wert egal, nur als Signal
} PauseMessage;

// Quadstar states used in the packet
typedef enum {
  STATE_IDLE = 0,
  STATE_ACTIVE = 1,
  STATE_RECOVERY = 2,
} QuadstarState;

// Packet that will be sent over ESP-NOW. Use fixed-size fields and a
// fixed-length C string for status to keep the packet binary-safe.
typedef struct __attribute__((packed)) quadstarData {
  int8_t state;      // use int8_t to keep size predictable (stores QuadstarState)
  int16_t setpoint_roll;      // -100 .. 100
  int16_t setpoint_pitch;     // -100 .. 100
  int16_t setpoint_yaw;       // -100 .. 100
  int16_t thrust;    // 0 .. 100 (or as used by the controller)
  int16_t setpoint_delta_north;
  int16_t setpoint_delta_east;
  int16_t setpoint_delta_down;
  uint8_t battery;   // 0 .. 100 %
  char statusMessage[64];
} QuadcomPacket;

uint8_t peerAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // Broadcast

unsigned long pauseUntil = 0;

// Global target speed for PWM control (accessible from callbacks)
int targetSpeed = 0;  // 0 ... 1000
// Global roll/pitch setpoints (-100 .. 100)
int targetRoll = 0;
int targetPitch = 0;

// UART2 RX buffer for incoming data from STM32 (or other MCU)
static const size_t UART2_RX_BUF_SZ = 512;
static uint8_t uart2_rx_buf[UART2_RX_BUF_SZ];
static size_t uart2_rx_idx = 0;

// CRC16-CCITT (poly 0x1021) helper
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

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  // If it's a pause message, handle as before
  if (len == sizeof(PauseMessage)) {
    pauseUntil = millis() + 1000;
    digitalWrite(LED_GPIO, LOW);
    ledState = false;
    Serial.println("Pause durch Empfang ausgelöst!");
    return;
  }

  // If it's a QuadcomPacket, parse and apply values
  if (len == sizeof(QuadcomPacket)) {
    QuadcomPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    targetSpeed = pkt.thrust;
    targetRoll = pkt.setpoint_roll;
    targetPitch = pkt.setpoint_pitch;
    Serial.print("[RX] thrust: "); Serial.println(pkt.thrust);
    Serial.print("[RX] setpoint_roll: "); Serial.println(pkt.setpoint_roll);
    Serial.print("[RX] setpoint_pitch: "); Serial.println(pkt.setpoint_pitch);
    // Optional: handle state or battery
      // New mapping requested:
      // - STATE_RECOVERY (and STATE_ERROR) -> blink LED
      // - STATE_IDLE -> LED off
      // - STATE_ACTIVE -> LED permanently ON
      if (pkt.state == STATE_RECOVERY) {
        // recovery -> blink
        errorState = true;
        idleState = false; // not in permanent-ON mode
        // start blinking immediately
        lastBlink = millis() - errorBlinkInterval;
        Serial.println("[RX] STATE_RECOVERY empfangen: Blinkmodus aktiv");
      } else if (pkt.state == STATE_ACTIVE) {
        // active -> LED on permanently
        errorState = false;
        idleState = true; // reuse as 'permanent ON' flag
        digitalWrite(LED_GPIO, HIGH);
        ledState = true;
        Serial.println("[RX] STATE_ACTIVE empfangen: LED dauerhaft AN");
      } else if (pkt.state == STATE_IDLE) {
        // idle -> LED off
        errorState = false;
        idleState = false;
        digitalWrite(LED_GPIO, LOW);
        ledState = false;
        Serial.println("[RX] STATE_IDLE empfangen: LED aus");
      } else {
        // unknown: clear modes
        errorState = false;
        idleState = false;
      }
      // --- Send a small test/info message over hardware UART for debugging ---
      // This helps verify that the STM32 (or other MCU) receives a notification
      // whenever we get a packet via ESP-NOW.
      {
        String info = "[RX_TEST] len=" + String(len) + ", thrust=" + String(pkt.thrust) + "\n";
        Serial.print("[UART2 TX] "); Serial.print(info);
        // Safe write to Serial2 (hardware UART) - non-blocking print
        Serial2.print(info);
        // Additionally send a compact CSV line for easy parsing on the other side
        String csv = String(pkt.thrust) + "," + String(pkt.setpoint_roll) + "," + String(pkt.setpoint_pitch) + "\n";
        Serial2.print(csv);
      }
    return;
  }
}

const int ledPin = 2;

void setup() {
  Serial.begin(115200);
  Serial.println("Boot OK");
  // Initialize hardware UART (Serial2) for communication with STM32 or other MCU
  // Note: pins used here are the common ESP32 DevKit mapping (TX=17, RX=16).
  // Adjust UART_TX_PIN / UART_RX_PIN if your wiring differs.
  Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.println("Serial2 (hardware UART) initialized on TX=" + String(UART_TX_PIN) + " RX=" + String(UART_RX_PIN));
  pinMode(LED_GPIO, OUTPUT);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);

  // 🔧 Initialize PWM for ESC
  // Channel 0 = thrust
  ledcSetup(0, 490, 16);         // Channel 0, 490 Hz, 16-bit resolution
  ledcAttachPin(PWM_PIN, 0);
  // Channel 1 = roll
  ledcSetup(1, 490, 16);
  ledcAttachPin(ROLL_PWM_PIN, 1);
  // Channel 2 = pitch
  ledcSetup(2, 490, 16);
  ledcAttachPin(PITCH_PWM_PIN, 2);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init failed!");
    while (true) {}
  }

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  bool buttonState = digitalRead(BUTTON_GPIO);
  unsigned long now = millis();
  if (buttonState == LOW) {
    digitalWrite(LED_GPIO, LOW);
    Serial.println("Taster gedrueckt!");
    PauseMessage msg;
    msg.pause = 1;
    esp_now_send(peerAddress, (uint8_t*)&msg, sizeof(msg));
    Serial.println("Pause-Signal gesendet!");
    pauseUntil = now + 1000;
  }

  // Blinken nur wenn keine Pause aktiv
  if (now >= pauseUntil && buttonState != LOW) {
    if (idleState) {
      // active -> LED permanently on
      digitalWrite(LED_GPIO, HIGH);
      ledState = true;
    } else if (errorState) {
      // recovery/error blink
      unsigned long currentBlink = errorBlinkInterval;
      if (now - lastBlink >= currentBlink) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(LED_GPIO, ledState ? HIGH : LOW);
      }
    } else {
      // idle -> LED off
      digitalWrite(LED_GPIO, LOW);
      ledState = false;
    }
  } else if (now < pauseUntil) {
    digitalWrite(LED_GPIO, LOW);
    ledState = false;
  }

  // Benutzereingabe auslesen
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    Serial.print("[LOG] Rohdaten empfangen: ");
    Serial.println(msg);

  int thrust = 0, roll = 0, pitch = 0;
  int controller_flag = 0;
    // Prüfe, ob Kommas enthalten sind
    if (msg.indexOf(',') != -1) {
      int firstComma = msg.indexOf(',');
      int secondComma = msg.indexOf(',', firstComma + 1);
      if (firstComma != -1 && secondComma != -1) {
        int thirdComma = msg.indexOf(',', secondComma + 1);
        thrust = msg.substring(0, firstComma).toInt();
        roll = msg.substring(firstComma + 1, secondComma).toInt();
        if (thirdComma != -1) {
          // format: thrust,roll,pitch,controller_flag
          pitch = msg.substring(secondComma + 1, thirdComma).toInt();
          controller_flag = msg.substring(thirdComma + 1).toInt();
        } else {
          // format: thrust,roll,pitch
          pitch = msg.substring(secondComma + 1).toInt();
          controller_flag = 0;
        }
        targetSpeed = thrust;
        Serial.print("Joystick Geschwindigkeit: ");
        Serial.println(targetSpeed);
        Serial.print("Roll: ");
        Serial.println(roll);
        Serial.print("Pitch: ");
        Serial.println(pitch);
        Serial.print("Controller active: "); Serial.println(controller_flag);
      } else {
        Serial.print("Ungültige Nachricht: ");
        Serial.println(msg);
      }
    } else {
      // Fallback: Nur ein Wert
      thrust = msg.toInt();
      if (thrust >= 0 && thrust <= 1000) {
        targetSpeed = thrust;
        Serial.print("Joystick Geschwindigkeit: ");
        Serial.println(targetSpeed);
      } else {
        Serial.print("Ungültige Nachricht: ");
        Serial.println(msg);
      }
    }
    while (Serial.available()) Serial.read();
    Serial.println("[LOG] Buffer nach Lesen geleert.");
    // Sende das gesamte Packet via ESP-NOW
    {
  QuadcomPacket outPkt;
  memset(&outPkt, 0, sizeof(outPkt));
  // Determine state from controller flag (button).
  // controller_flag mapping from joystick:
  // 0 = IDLE, 1 = ACTIVE, 2 = RECOVERY
  if (controller_flag == 2) {
    outPkt.state = STATE_RECOVERY;
    // also set the local sender into recovery (blink) mode
    errorState = true;
    idleState = false;
    lastBlink = millis() - errorBlinkInterval;
    Serial.println("[TX] Recovery requested from joystick: local blink aktiviert");
  } else if (controller_flag == 1) {
    outPkt.state = STATE_ACTIVE;
    // local active: LED permanently on
    errorState = false;
    idleState = true;
    digitalWrite(LED_GPIO, HIGH);
    ledState = true;
    Serial.println("[TX] Active requested from joystick: LED dauerhaft AN");
  } else {
    outPkt.state = STATE_IDLE;
    // local idle: LED off
    errorState = false;
    idleState = false;
    digitalWrite(LED_GPIO, LOW);
    ledState = false;
    Serial.println("[TX] Idle requested from joystick: LED aus");
  }
      outPkt.setpoint_roll = roll;
      outPkt.setpoint_pitch = pitch;
      outPkt.setpoint_yaw = 0;
      outPkt.thrust = thrust;
      outPkt.setpoint_delta_north = 0;
      outPkt.setpoint_delta_east = 0;
      outPkt.setpoint_delta_down = 0;
      outPkt.battery = 100;
      snprintf(outPkt.statusMessage, sizeof(outPkt.statusMessage), "from-serial");

      esp_err_t res = esp_now_send(peerAddress, (uint8_t *)&outPkt, sizeof(outPkt));
      if (res == ESP_OK) {
        Serial.println("QuadcomPacket gesendet.");
      } else {
        Serial.print("Fehler beim Senden (esp_now_send): ");
        Serial.println(res);
      }
      // Always forward a compact CSV line over the hardware UART (Serial2)
      // so the main compute unit (STM32) receives the same control values
      // regardless of ESP-NOW forwarding success.
      {
        String info = "[TX_UART] thrust=" + String(outPkt.thrust) + "\n";
        Serial.print("[UART2 TX] "); Serial.print(info);
        Serial2.print(info);
        String csv = String(outPkt.thrust) + "," + String(outPkt.setpoint_roll) + "," + String(outPkt.setpoint_pitch) + "\n";
        Serial2.print(csv);
      }
    }
  }



  int pulse_us = map(targetSpeed, 0, 100, 1000, 2000);
  int period_us = 1000000 / 490;   // ≈ 2041 µs
  int duty = (pulse_us * 65535) / period_us;
  ledcWrite(0, duty);

  // Write roll + pitch PWM (map -100..100 -> 1000..2000us, center=1500us)
  int roll_pulse_us = map(targetRoll, -100, 100, 1000, 2000);
  int roll_duty = (roll_pulse_us * 65535) / period_us;
  ledcWrite(1, roll_duty);

  int pitch_pulse_us = map(targetPitch, -100, 100, 1000, 2000);
  int pitch_duty = (pitch_pulse_us * 65535) / period_us;
  ledcWrite(2, pitch_duty);

  // --- Read from hardware UART (Serial2) and forward Quadcom frames via ESP-NOW ---
  while (Serial2.available()) {
    int c = Serial2.read();
    if (c < 0) break;
    // append to buffer if space
    if (uart2_rx_idx < UART2_RX_BUF_SZ) {
      uart2_rx_buf[uart2_rx_idx++] = (uint8_t)c;
    } else {
      // overflow - reset buffer
      uart2_rx_idx = 0;
    }

    // Try to parse frames in buffer
    // Frame format expected: 0xAA 0x55 | len_lo | len_hi | payload[len] | crc_lo | crc_hi
    size_t pos = 0;
    while (pos + 6 <= uart2_rx_idx) {
      // search header
      if (uart2_rx_buf[pos] != 0xAA || uart2_rx_buf[pos+1] != 0x55) {
        pos++;
        continue;
      }
      // read length
      uint16_t payload_len = (uint16_t)uart2_rx_buf[pos+2] | ((uint16_t)uart2_rx_buf[pos+3] << 8);
      size_t full_len = 2 + 2 + payload_len + 2;
      if (pos + full_len > uart2_rx_idx) {
        // incomplete frame yet
        break;
      }
      // compute CRC over payload
      const uint8_t *payload_ptr = &uart2_rx_buf[pos + 4];
      uint16_t recv_crc = (uint16_t)uart2_rx_buf[pos + 4 + payload_len] | ((uint16_t)uart2_rx_buf[pos + 4 + payload_len + 1] << 8);
      uint16_t calc_crc = crc16_ccitt(payload_ptr, payload_len);
      if (calc_crc == recv_crc) {
        // valid frame -> forward payload via ESP-NOW as-is
        esp_err_t res = esp_now_send(peerAddress, payload_ptr, payload_len);
        if (res == ESP_OK) {
          Serial.print("[UART2->ESPNOW] forwarded payload len="); Serial.println(payload_len);
        } else {
          Serial.print("[UART2->ESPNOW] forward failed: "); Serial.println(res);
        }
      } else {
        Serial.print("[UART2] CRC mismatch, recv=0x"); Serial.print(recv_crc, HEX);
        Serial.print(" calc=0x"); Serial.println(calc_crc, HEX);
      }
      // consume this frame from buffer (move remaining bytes to front)
      size_t rem = uart2_rx_idx - (pos + full_len);
      if (rem > 0) memmove(uart2_rx_buf, &uart2_rx_buf[pos + full_len], rem);
      uart2_rx_idx = rem;
      pos = 0; // restart parsing at buffer start
    }
    // Also handle simple ASCII line debugging: if newline present, print and clear up to newline
    for (size_t i = 0; i < uart2_rx_idx; ++i) {
      if (uart2_rx_buf[i] == '\n' || uart2_rx_buf[i] == '\r') {
        // print the preceding bytes as a line
        if (i > 0) {
          char line[256];
          size_t copy_len = (i < sizeof(line)-1) ? i : (sizeof(line)-1);
          memcpy(line, uart2_rx_buf, copy_len);
          line[copy_len] = '\0';
          Serial.print("[UART2 RX LINE] "); Serial.println(line);
        }
        // remove consumed bytes including the newline
        size_t rem = uart2_rx_idx - (i+1);
        if (rem > 0) memmove(uart2_rx_buf, &uart2_rx_buf[i+1], rem);
        uart2_rx_idx = rem;
        break; // process one line per loop, rest will be handled later
      }
    }
  }

  delay(10);
}
