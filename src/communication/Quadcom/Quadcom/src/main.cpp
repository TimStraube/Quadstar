

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>



#define LED_GPIO 2
#define BUTTON_GPIO 18

const unsigned long blinkInterval = 500;
unsigned long lastBlink = 0;
bool ledState = false;

// Dummy-Struktur für das Pause-Signal
typedef struct __attribute__((packed)) {
  uint8_t pause; // Wert egal, nur als Signal
} PauseMessage;

// MAC-Adresse des Partners (hier als Platzhalter, muss ggf. angepasst werden)
uint8_t peerAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // Broadcast


unsigned long pauseUntil = 0;
// Callback für empfangene Nachrichten
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(PauseMessage)) {
    pauseUntil = millis() + 1000;
    digitalWrite(LED_GPIO, LOW);
    ledState = false;
    Serial.println("Pause durch Empfang ausgelöst!");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Boot OK");
  pinMode(LED_GPIO, OUTPUT);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);

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
      Serial.print("LED: ");
      Serial.println(ledState ? "AN" : "AUS");
    }
  } else if (now < pauseUntil) {
    digitalWrite(LED_GPIO, LOW);
    ledState = false;
  }
  delay(10);
}
