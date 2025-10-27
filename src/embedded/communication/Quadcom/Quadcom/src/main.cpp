#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#define LED_GPIO 2
#define BUTTON_GPIO 18
#define PWM_PIN 21

const unsigned long blinkInterval = 500;
unsigned long lastBlink = 0;
bool ledState = false;

// Dummy-Struktur für das Pause-Signal
typedef struct __attribute__((packed)) {
  uint8_t pause; // Wert egal, nur als Signal
} PauseMessage;

uint8_t peerAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // Broadcast

unsigned long pauseUntil = 0;

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(PauseMessage)) {
    pauseUntil = millis() + 1000;
    digitalWrite(LED_GPIO, LOW);
    ledState = false;
    Serial.println("Pause durch Empfang ausgelöst!");
  }
}

const int ledPin = 2;
int targetSpeed = 0;  // 0 ... 1000

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
    if (now - lastBlink >= blinkInterval) {
      lastBlink = now;
      ledState = !ledState;
      digitalWrite(LED_GPIO, ledState ? HIGH : LOW);
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
    // Prüfe, ob Kommas enthalten sind
    if (msg.indexOf(',') != -1) {
      int firstComma = msg.indexOf(',');
      int secondComma = msg.indexOf(',', firstComma + 1);
      if (firstComma != -1 && secondComma != -1) {
        thrust = msg.substring(0, firstComma).toInt();
        roll = msg.substring(firstComma + 1, secondComma).toInt();
        pitch = msg.substring(secondComma + 1).toInt();
        targetSpeed = thrust;
        Serial.print("Joystick Geschwindigkeit: ");
        Serial.println(targetSpeed);
        Serial.print("Roll: ");
        Serial.println(roll);
        Serial.print("Pitch: ");
        Serial.println(pitch);
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
  }



  int pulse_us = map(targetSpeed, 0, 100, 1000, 2000);
  int period_us = 1000000 / 490;   // ≈ 2041 µs
  int duty = (pulse_us * 65535) / period_us;
  ledcWrite(0, duty);

  delay(10);
}
