#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define LED_GPIO 2
#define BUTTON_GPIO 18
#define PWM_PIN 21
// UART pins to STM32 (change if your wiring differs)
#define STM32_RX_PIN 16 // ESP32 RX pin (connect to STM32 TX)
#define STM32_TX_PIN 17 // ESP32 TX pin (connect to STM32 RX)

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

// forwarder prototype - declared before usage in onReceive / other places
static void forward_to_stm32(const uint8_t *payload, uint16_t payload_len);

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  // If it's a pause message, handle as before
  if (len == sizeof(PauseMessage)) {
    pauseUntil = millis() + 1000;
    digitalWrite(LED_GPIO, LOW);
    ledState = false;
    Serial.println("Pause durch Empfang ausgelöst!");
    // forward pause to STM32
    // small payload: the PauseMessage bytes
    forward_to_stm32(data, sizeof(PauseMessage));
    return;
  }

  // If it's a QuadcomPacket, parse and apply values
  if (len == sizeof(QuadcomPacket)) {
    QuadcomPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    targetSpeed = pkt.thrust;
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
    // forward full packet to STM32 as well
    forward_to_stm32((const uint8_t *)&pkt, sizeof(pkt));
    return;
  }
}

// CRC-16-CCITT (0x1021) - returns CRC
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

// Frame: 0xAA 0x55 | len_lo | len_hi | payload | crc_lo | crc_hi
static void forward_to_stm32(const uint8_t *payload, uint16_t payload_len) {
  if (!Serial2) return; // Serial2 not initialized
  uint8_t header[4];
  header[0] = 0xAA;
  header[1] = 0x55;
  header[2] = payload_len & 0xFF;
  header[3] = (payload_len >> 8) & 0xFF;
  uint16_t crc = crc16_ccitt(payload, payload_len);
  Serial2.write(header, sizeof(header));
  Serial2.write(payload, payload_len);
  uint8_t crcbuf[2]; crcbuf[0] = crc & 0xFF; crcbuf[1] = (crc >> 8) & 0xFF;
  Serial2.write(crcbuf, 2);
  Serial2.flush();
  Serial.print("Forwarded to STM32 (bytes): "); Serial.println(payload_len);
}

const int ledPin = 2;

void setup() {
  Serial.begin(115200);
  // initialize hardware UART to STM32
  Serial2.begin(115200, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  Serial.println("Serial2 to STM32 initialized");
  Serial.println("Boot OK");
  pinMode(LED_GPIO, OUTPUT);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);

  // 🔧 Initialize PWM for ESC
  ledcSetup(0, 490, 16);         // Channel 0, 490 Hz (2.04ms), 16-bit resolution
  ledcAttachPin(PWM_PIN, 0);

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
        // forward locally-sent packet to STM32 as well
        forward_to_stm32((const uint8_t *)&outPkt, sizeof(outPkt));
      } else {
        Serial.print("Fehler beim Senden (esp_now_send): ");
        Serial.println(res);
      }
    }
  }



  int pulse_us = map(targetSpeed, 0, 100, 1000, 2000);
  int period_us = 1000000 / 490;   // ≈ 2041 µs
  int duty = (pulse_us * 65535) / period_us;
  ledcWrite(0, duty);

  delay(10);
}
