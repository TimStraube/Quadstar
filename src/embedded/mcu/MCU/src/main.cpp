#include <Arduino.h>
#include <HardwareSerial.h>

// Ensure a hardware serial instance for USART2 (PA2/PA3) is available.
// Some STM32 Arduino variants already provide Serial1; if not, we create one.
// Construct a HardwareSerial bound to USART2 so we can use Serial1 in the sketch.
HardwareSerial Serial1(USART2);

// Simple UART test sketch
// - uses Serial1 (USART2/PA2=TX, PA3=RX on Nucleo boards)
// - on any received line toggles the on-board LED and replies with "[STM32_TEST]\n"

void setup() {
  // Debug console over USB-CDC (if available)
  Serial.begin(115200);
  /* Don't block here waiting for the USB CDC to be opened by the host.
    If the monitor isn't connected, while(!Serial) would hang the board and
    prevent the UART test from running. Wait a short time for enumeration
    and continue regardless. */
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 200) { delay(2); }
  Serial.println("MCU UART test - boot");

  // Hardware UART to test with external MCU (ESP32)
  // On Nucleo F4 with Arduino core, Serial1 typically maps to USART2 (PA2/PA3)
  Serial1.begin(115200);
  Serial.println("Serial1 started @115200");

  // LED pin (Arduino macro) - should map to the on-board LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // small timeout for readStringUntil
  Serial1.setTimeout(50);
}

void loop() {
  // Non-blocking check for a full line on Serial1
  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      // Received line on Serial1 (external MCU). We reply with an ACK but
      // avoid toggling here so the LED can be controlled by the heartbeat
      // blink below (keeps behaviour predictable).

      // Send ACK back
      Serial1.print("[STM32_TEST]\n");

      // Also echo to USB console for debugging
      Serial.print("RX Serial1: ");
      Serial.println(line);
      Serial.println("-> Sent [STM32_TEST]");
    }
  }

  // keep a tiny delay to yield
  // Heartbeat blink (non-blocking)
  static unsigned long lastBlink = 0;
  const unsigned long blinkInterval = 500; // ms
  unsigned long now = millis();
  if (now - lastBlink >= blinkInterval) {
    lastBlink = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  delay(5);
}
