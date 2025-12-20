#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "controller.h"
#include "sensor.h"

// Heartbeat LED pin (PH1 used previously)
const int HEARTBEAT_PIN = PH1;
// Visual feedback LED (PWM capable pin). Change if this pin conflicts with your board wiring.
const int LED_PWM_PIN = PC7;

// PWM input pins (connect NodeMCU/ESP32 PWM outputs here)
// Change these if you wire to different MCU pins.
const int PWM_IN_THR_PIN = PC0;   // thrust PWM input
const int PWM_IN_ROLL_PIN = PC1;  // roll PWM input
const int PWM_IN_PITCH_PIN = PC2; // pitch PWM input

// PWM capture raw pulse widths in microseconds (updated in ISRs)
volatile unsigned long pwm_thrust_us = 0;
volatile unsigned long pwm_roll_us = 0;
volatile unsigned long pwm_pitch_us = 0;

// internal rise timestamps
volatile unsigned long pwm_thrust_rise = 0;
volatile unsigned long pwm_roll_rise = 0;
volatile unsigned long pwm_pitch_rise = 0;

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
// motor_pwm for TIM1-based regulator
static double motor_pwm[4] = {0,0,0,0};
// Default PWM limits set to standard hobby servo/ESC pulse widths (µs)
float pwmunteregrenze = 1000.0f; // 1000 µs
float pwmoberegrenze = 2000.0f; // 2000 µs
float skalar = 0.08f; // will be overwritten in loop to (pwmoberegrenze-pwmunteregrenze)/1000
uint8_t regleran = 0;


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
static void control_step() {
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

  // Update motor_pwm array (current normalized motor commands)
  for (int i = 0; i < 4; ++i) {
    motor_pwm[i] = (double)constrain(m[i], 0.0f, 1.0f);
  }

  // Compute skalar as in original
  float skalar_local = (pwmoberegrenze - pwmunteregrenze) / 1000.0f;

  // Apply to hardware
  control(motor_pwm, setpoint_thrust, skalar_local, pwmunteregrenze, pwmoberegrenze);
}
 
void setup() {
  // Heartbeat LED
  pinMode(HEARTBEAT_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_PIN, LOW);

  // Configure PWM input pins and attach interrupts to measure pulse widths.
  // We use CHANGE interrupts and micros() timestamps to measure pulse length in µs.
  pinMode(PWM_IN_THR_PIN, INPUT);
  pinMode(PWM_IN_ROLL_PIN, INPUT);
  pinMode(PWM_IN_PITCH_PIN, INPUT);
  // Attach change interrupts. Each handler decides if rising or falling.
  attachInterrupt(digitalPinToInterrupt(PWM_IN_THR_PIN), []() {
    // thrust pin change handler
    if (digitalRead(PWM_IN_THR_PIN)) {
      pwm_thrust_rise = micros();
    } else {
      unsigned long now = micros();
      unsigned long dur = now - pwm_thrust_rise;
      // sanity clamp: ignore implausible long pulses (> 50000 µs)
      if (dur < 50000U) pwm_thrust_us = dur; else pwm_thrust_us = 0;
    }
  }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_IN_ROLL_PIN), []() {
    if (digitalRead(PWM_IN_ROLL_PIN)) {
      pwm_roll_rise = micros();
    } else {
      unsigned long now = micros();
      unsigned long dur = now - pwm_roll_rise;
      if (dur < 50000U) pwm_roll_us = dur; else pwm_roll_us = 0;
    }
  }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_IN_PITCH_PIN), []() {
    if (digitalRead(PWM_IN_PITCH_PIN)) {
      pwm_pitch_rise = micros();
    } else {
      unsigned long now = micros();
      unsigned long dur = now - pwm_pitch_rise;
      if (dur < 50000U) pwm_pitch_us = dur; else pwm_pitch_us = 0;
    }
  }, CHANGE);

  // Enable USB serial for debug output (non-blocking prints)
  Serial.begin(115200);
  Serial.println("MCU UART test - boot");

  // initialize sensor module (I2C, MPU6050, offsets)
  sensor_init();
  // Configure LED PWM pin as output (analogWrite will initialize timers)
  pinMode(LED_PWM_PIN, OUTPUT);
  // Configure motor pins (PA8..PA11 used on the original board). Change if needed
  controllerConfigurePins(PA8, PA9, PA10, PA11);

  // Set thrust to 55% by default for quick spin test (0..100)
  joystick_thrust_cmd = 55;
}

void loop() {
  // --- Non-blocking UART1 CSV receive (thrust,roll,pitch) ---
  static const size_t RX_BUF_LEN = 128;
  static char rxBuf[RX_BUF_LEN];
  static size_t rxLen = 0;
 

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

  // --- PWM input mapping: map captured pulse widths (µs) to joystick commands
  // Read volatile pulse widths atomically
  noInterrupts();
  unsigned long thr_us = pwm_thrust_us;
  unsigned long rol_us = pwm_roll_us;
  unsigned long pit_us = pwm_pitch_us;
  interrupts();

  // Map thrust: expect 1000..2000 µs => 0..100
  if (thr_us >= 900 && thr_us <= 2100) {
    int mapped = map((int)thr_us, 1000, 2000, 0, 100);
    joystick_thrust_cmd = constrain(mapped, 0, 100);
  } else {
    // no valid PWM on thrust pin -> leave joystick_thrust_cmd as-is (may be -1)
  }

  // Map roll/pitch: expect 1000..2000 µs => -100..100
  if (rol_us >= 900 && rol_us <= 2100) {
    int mapped = map((int)rol_us, 1000, 2000, -100, 100);
    joystick_roll_cmd = constrain(mapped, -100, 100);
  }
  if (pit_us >= 900 && pit_us <= 2100) {
    int mapped = map((int)pit_us, 1000, 2000, -100, 100);
    joystick_pitch_cmd = constrain(mapped, -100, 100);
  }

  // Periodic debug print of pulse widths and mapped setpoints
  static unsigned long lastPwmPrint = 0;
  if (millis() - lastPwmPrint >= 500UL) {
    lastPwmPrint = millis();
    Serial.print("PWM(us): T="); Serial.print(thr_us);
    Serial.print(", R="); Serial.print(rol_us);
    Serial.print(", P="); Serial.print(pit_us);
    Serial.print("  -> setpoints: ");
    Serial.print("T="); Serial.print(joystick_thrust_cmd);
    Serial.print(", R="); Serial.print(joystick_roll_cmd);
    Serial.print(", P="); Serial.println(joystick_pitch_cmd);
  }
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
  control_step();

  // Replace the slow sinusoidal ramp with a faster linear ramp for thrust
  static float test_thrust = 0.0f;
  static bool increasing = true;

  // Update test_thrust in loop
  if (joystick_thrust_cmd < 0) {
      if (increasing) {
          test_thrust += 20.0f; // Increase faster
          if (test_thrust >= 100.0f) {
              test_thrust = 100.0f;
              increasing = false;
          }
      } else {
          test_thrust -= 20.0f; // Decrease faster
          if (test_thrust <= 0.0f) {
              test_thrust = 0.0f;
              increasing = true;
          }
      }
      joystick_thrust_cmd = (int)test_thrust;
  }

  delay(10);
}
