#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <math.h>
#include "controller.h"
#include "sensor.h"

// Provide a HardwareSerial instance bound to USART2 (PA2/PA3) so we can
// read the CSV lines coming from the ESP32 over the hardware UART.
HardwareSerial Serial1(USART2);

// Minimal firmware: heartbeat LED + PWM input measurement

// Heartbeat LED pin (PH1 used previously)
const int HEARTBEAT_PIN = PH1;
// Visual feedback LED (PWM capable pin). Change if this pin conflicts with your board wiring.
const int LED_PWM_PIN = PC7;

// Measured thrust/roll/pitch from UART CSV
// measuredThrust: 0..100, -1 = invalid
volatile int measuredThrust = -1;
// measuredRoll / measuredPitch: -100..100, -127 = invalid
// Flag set when new UART values were parsed (used for USB debug prints)
volatile bool uartUpdated = false;
// joystick command inputs (from Serial1 CSV). -1 means no command yet.
volatile int joystick_thrust_cmd = -1;
volatile int joystick_roll_cmd = 0;
volatile int joystick_pitch_cmd = 0;
// motor_pwm history for TIM1-based regulator
static double motor_pwm[4] = {0,0,0,0};
static double motor_pwm_tminus1[4] = {0,0,0,0};
static double motor_pwm_tminus2[4] = {0,0,0,0};
float pwmunteregrenze = 800.0f;
float pwmoberegrenze = 880.0f;
float skalar = 0.08f; // will be overwritten in loop to (pwmoberegrenze-pwmunteregrenze)/1000
uint8_t regleran = 0;

// Toggle to disable hardware Serial1 (USART2) when debugging I2C bus issues
const bool ENABLE_SERIAL1 = false;


// Simple ported regler helper: for now apply uniform thrust from joystick
// to TIM1 so we can validate PWM output. This will be extended later
// to run the full regler logic (rate controller + mixer).
// Ported, simplified regelschritt: rate-controller + mixer -> motor_pwm -> control
// Assumptions / simplifications made:
// - Use fused angles `angleRoll`/`anglePitch` computed in the main loop as attitude.
// - Desired angular rates are taken from joystick inputs: roll_cmd/50, pitch_cmd/50 (same factor as original).
// - A simple P (or PD with small D) rate controller is used to compute control efforts for roll/pitch.
// - Mixer follows an X-quad layout and combines thrust + roll/pitch efforts.
// - motor_pwm values are normalized [0..1] and control is called with regleran=true.
static void regelschritt_ported() {
  // Read joystick inputs atomically (thrust: 0..100, roll/pitch: -100..100)
  int thrust_cmd, roll_cmd, pitch_cmd;
  noInterrupts();
  thrust_cmd = joystick_thrust_cmd;
  roll_cmd = joystick_roll_cmd;
  pitch_cmd = joystick_pitch_cmd;
  interrupts();

  // If no joystick/serial input present, use fixed test constants so we can
  // validate PWM output without relying on Serial1/USB input.
  if (thrust_cmd < 0) {
    thrust_cmd = 50;   // 50% thrust test
    roll_cmd = 0;      // 0% roll
    pitch_cmd = 10;    // 10% pitch
    // Note: no early return — continue with controller using these constants
  }

  // Time delta estimation using imuLastMs (set when IMU read). If not available, assume 10ms.
  static unsigned long last_ms = 0;
  unsigned long now_ms = imuLastMs ? imuLastMs : millis();
  float dt = 0.01f;
  if (last_ms != 0) {
    long diff = (long)(now_ms - last_ms);
    if (diff > 0) dt = diff / 1000.0f;
  }

  // Estimate angular rates (deg/s) from fused angles
  static float lastAngleRoll = 0.0f;
  static float lastAnglePitch = 0.0f;
  float meas_rate_roll = 0.0f;
  float meas_rate_pitch = 0.0f;
  meas_rate_roll = (angleRoll - lastAngleRoll) / max(0.001f, dt);
  meas_rate_pitch = (anglePitch - lastAnglePitch) / max(0.001f, dt);
  lastAngleRoll = angleRoll;
  lastAnglePitch = anglePitch;
  last_ms = now_ms;

  // Simple angle-PID controller
  // Map joystick percent (-100..100) to desired angle in degrees. Choose a
  // conservative max tilt (e.g. 30 deg) so 100% -> 30deg, 10% -> 3deg.
  const float maxTiltDeg = 30.0f;
  float desired_angle_roll = (float)roll_cmd * maxTiltDeg / 100.0f;
  float desired_angle_pitch = (float)pitch_cmd * maxTiltDeg / 100.0f;

  // PID gains (conservative). Tune these later on the hardware.
  const float Kp_roll = 0.9f;
  const float Ki_roll = 0.02f;
  const float Kd_roll = 0.01f;
  const float Kp_pitch = 0.9f;
  const float Ki_pitch = 0.02f;
  const float Kd_pitch = 0.01f;

  // PID state (static across calls)
  static float integ_roll = 0.0f;
  static float integ_pitch = 0.0f;
  static float last_err_roll = 0.0f;
  static float last_err_pitch = 0.0f;

  // Error is desired angle minus measured fused angle
  float err_roll = desired_angle_roll - angleRoll;
  float err_pitch = desired_angle_pitch - anglePitch;

  // Integrator with simple anti-windup clamp
  integ_roll += err_roll * dt;
  integ_pitch += err_pitch * dt;
  const float integ_limit = 20.0f; // clamp integrator to prevent runaway
  if (integ_roll > integ_limit) integ_roll = integ_limit;
  if (integ_roll < -integ_limit) integ_roll = -integ_limit;
  if (integ_pitch > integ_limit) integ_pitch = integ_limit;
  if (integ_pitch < -integ_limit) integ_pitch = -integ_limit;

  // Derivative (error derivative)
  float derr_roll = (err_roll - last_err_roll) / max(0.0001f, dt);
  float derr_pitch = (err_pitch - last_err_pitch) / max(0.0001f, dt);
  last_err_roll = err_roll;
  last_err_pitch = err_pitch;

  // PID outputs (these are torque-like contributions used by the mixer)
  float ctrl_roll = Kp_roll * err_roll + Ki_roll * integ_roll + Kd_roll * derr_roll;
  float ctrl_pitch = Kp_pitch * err_pitch + Ki_pitch * integ_pitch + Kd_pitch * derr_pitch;

  // Normalize thrust to 0..1
  float setpoint_thrust = (float)thrust_cmd / 100.0f;

  // Mixer (X configuration). attitude_gain controls how strongly roll/pitch efforts affect motors
  const float attitude_gain = 0.4f;
  float m[4];
  // m1 = thrust + attitude_gain*( pitch + roll )
  m[0] = setpoint_thrust + attitude_gain * ( ctrl_pitch + ctrl_roll );
  // m2 = thrust + attitude_gain*( pitch - roll )
  m[1] = setpoint_thrust + attitude_gain * ( ctrl_pitch - ctrl_roll );
  // m3 = thrust + attitude_gain*( -pitch - roll )
  m[2] = setpoint_thrust + attitude_gain * ( -ctrl_pitch - ctrl_roll );
  // m4 = thrust + attitude_gain*( -pitch + roll )
  m[3] = setpoint_thrust + attitude_gain * ( -ctrl_pitch + ctrl_roll );

  // Clamp and renormalize to 0..1
  float minv = m[0], maxv = m[0];
  for (int i = 1; i < 4; ++i) { if (m[i] < minv) minv = m[i]; if (m[i] > maxv) maxv = m[i]; }
  if (minv < 0.0f || maxv > 1.0f) {
    float span = maxv - minv;
    if (span <= 0.0f) span = 1.0f;
    for (int i = 0; i < 4; ++i) m[i] = (m[i] - minv) / span;
  }

  // Update motor_pwm history arrays (they are globals)
  for (int i = 0; i < 4; ++i) {
    motor_pwm_tminus2[i] = motor_pwm_tminus1[i];
    motor_pwm_tminus1[i] = motor_pwm[i];
    motor_pwm[i] = (double)constrain(m[i], 0.0f, 1.0f);
  }

  // Compute skalar as in original
  float skalar = (pwmoberegrenze - pwmunteregrenze) / 1000.0f;

  // Apply to hardware
  control(motor_pwm, motor_pwm_tminus1, motor_pwm_tminus2, setpoint_thrust, skalar, pwmunteregrenze, pwmoberegrenze);
}
 
void setup() {
  // Heartbeat LED
  pinMode(HEARTBEAT_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_PIN, LOW);

  // Configure PWM input pin and attach interrupt to measure pulse width.
  // PWM input removed — receiving thrust/roll/pitch via UART CSV

  // Enable USB serial for debug output (non-blocking prints)
  Serial.begin(115200);
  Serial.println("MCU UART test - boot");

  // Safe Serial1 initialization: optionally enable Serial1. When
  // debugging I2C, set ENABLE_SERIAL1=false to avoid any UART pin conflicts.
  if (ENABLE_SERIAL1) {
    // set PA2/PA3 to pull-down, wait briefly and then start the UART.
    pinMode(PA2, INPUT_PULLDOWN);
    pinMode(PA3, INPUT_PULLDOWN);
    delay(50);
    Serial1.begin(115200);
    Serial1.setTimeout(0);
    Serial.println("Serial1 started @115200");
  } else {
    Serial.println("Serial1 is DISABLED (I2C debug mode)");
  }
  // initialize sensor module (I2C, MPU6050, offsets)
  sensor_init();
  // Configure LED PWM pin as output (analogWrite will initialize timers)
  pinMode(LED_PWM_PIN, OUTPUT);
  // Configure motor pins (PA8..PA11 used on the original board). Change if needed
  controllerConfigurePins(PA8, PA9, PA10, PA11);
}

void loop() {
  // --- Non-blocking UART1 CSV receive (thrust,roll,pitch) ---
  static const size_t RX_BUF_LEN = 128;
  static char rxBuf[RX_BUF_LEN];
  static size_t rxLen = 0;
  if (ENABLE_SERIAL1 && Serial1) {
    int rxProcessed = 0;
    const int MAX_RX_PER_LOOP = 32;
    while (Serial1.available() > 0 && rxProcessed < MAX_RX_PER_LOOP) {
      int c = Serial1.read();
      rxProcessed++;
      if (c < 0) break;
      // Raw-byte debug: print each received byte in hex (very short)
      // This helps verify that bytes physically arrive on PA3.
      Serial.print("B:");
      if (c < 16) Serial.print('0');
      Serial.print(c, HEX);
      Serial.print(' ');
      if (c == '\r') continue;
      if (c == '\n') {
        rxBuf[rxLen] = '\0';
        if (rxLen > 0) {
          int t = 0, r = 0, p = 0;
          int got = sscanf(rxBuf, "%d,%d,%d", &t, &r, &p);
          // Only update values we parsed; keep everything atomic via interrupts
          noInterrupts();
          if (got >= 1) {
            // clamp thrust to sensible range (0..100)
            if (t < 0) t = 0;
            if (t > 100) t = 100;
            joystick_thrust_cmd = t;
          }
          if (got >= 2) {
            if (r < -100) r = -100;
            if (r > 100) r = 100;
            joystick_roll_cmd = r;
          }
          if (got >= 3) {
            if (p < -100) p = -100;
            if (p > 100) p = 100;
            joystick_pitch_cmd = p;
          }
          interrupts();
          // Immediate USB debug print of the raw line so you see it right away
          Serial.print("[Serial1 LINE] ");
          Serial.println(rxBuf);
          // Send a simple ACK back to the sender (ESP32) so it can detect a reply
          Serial1.print("ACK\n");
          // Also set the delayed debug flag (used for periodic full-value prints)
          uartUpdated = true;
        }
        rxLen = 0;
      } else {
        if (rxLen < RX_BUF_LEN - 1) rxBuf[rxLen++] = (char)c;
        else rxLen = 0; // overflow -> resync
      }
    }
  }

  // Blink LED slowly (~1s on / 1s off) using non-blocking timing
  // static bool ledState = false;
  // static unsigned long lastBlink = 0;
  // const unsigned long blinkInterval = 1000; // ms
  // unsigned long now = millis();
  // if (now - lastBlink >= blinkInterval) {
  //   lastBlink = now;
  //   ledState = !ledState;
  //   digitalWrite(HEARTBEAT_PIN, ledState ? HIGH : LOW);
  // }

  // No PWM sampling — thrust/roll/pitch come via UART CSV

  // small yield
  // LED brightness proportional to measuredThrust (0..100 -> 0..255)
  // Update sensor state (fused angles + measured roll/pitch)
  sensor_loop();
  // Periodic fused-angle print for debug
  static unsigned long lastImuPrint = 0;
  if (millis() - lastImuPrint >= 1000UL) {
    Serial.print("IMU angle (deg): Roll="); Serial.print(angleRoll, 2);
    Serial.print(", Pitch="); Serial.println(anglePitch, 2);
    lastImuPrint = millis();
  }

  // Non-blocking USB debug print when we got new UART values
  static unsigned long lastUsbPrint = 0;
  const unsigned long USB_PRINT_INTERVAL = 500UL; // ms
  if (uartUpdated && (millis() - lastUsbPrint >= USB_PRINT_INTERVAL)) {
    noInterrupts();
    int t = measuredThrust;
    int r = measuredRoll;
    int p = measuredPitch;
    uartUpdated = false;
    interrupts();
    Serial.print("[Serial1 RX] ");
    Serial.print(t);
    Serial.print(",");
    Serial.print(r);
    Serial.print(",");
    Serial.println(p);
    lastUsbPrint = millis();
  }
  int localThrust;
  noInterrupts();
  // use joystick command as source for thrust to drive the regulator
  localThrust = joystick_thrust_cmd;
  interrupts();
  if (localThrust >= 0) {
    int duty = map(localThrust, 0, 100, 0, 255);
    analogWrite(HEARTBEAT_PIN, duty);
  } else {
    // No thrust available: blink the heartbeat LED (non-blocking)
    static unsigned long lastBlink = 0;
    static bool blinkState = false;
    const unsigned long blinkInterval = 1000UL; // ms
    unsigned long now = millis();
    if (now - lastBlink >= blinkInterval) {
      lastBlink = now;
      blinkState = !blinkState;
    }
    // Use a visible on-level when blinking (half brightness)
    analogWrite(HEARTBEAT_PIN, blinkState ? 128 : 0);
  }

  // Always run the control step (it will use default test constants when no
  // joystick/serial input is present). This ensures motors get updated even
  // when Serial input is not available.
  regelschritt_ported();

  delay(10);
}
