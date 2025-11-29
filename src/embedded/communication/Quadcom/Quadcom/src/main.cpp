#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define LED_GPIO 2
#define BUTTON_GPIO 18
#define PWM_PIN 21

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
    return;
  }
}

const int ledPin = 2;

void setup() {
  Serial.begin(115200);
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
