/* sensor.cpp
 * Moved MPU6050 / IMU related code from main.cpp into a separate module.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "sensor.h"

// --- MPU6050 / IMU support (I2C on PB9/PB8 for Nucleo F4)
bool imuAvailable = false;
uint8_t MPU_ADDR = 0x68; // will be updated by scanner if 0x69 found
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t GYRO_XOUT_H = 0x43;
const uint8_t ACCEL_CONFIG = 0x1C;

float gyroXoff = 0, gyroYoff = 0, gyroZoff = 0;
unsigned long imuLastMs = 0;
float angleRoll = 0, anglePitch = 0;
const float alpha = 0.98f; // complementary filter
// runtime accel scale (LSB per g). Will be set in init after configuring/reading ACCEL_CONFIG
float accScale = 16384.0f; // default assume +/-2g until proven otherwise

static bool useSoftI2C = false;

volatile int measuredRoll = -127;
volatile int measuredPitch = -127;

// Forward declarations for bit-bang helpers
static void i2c_bb_delay();
static void i2c_bb_start();
static void i2c_bb_stop();
static bool i2c_bb_write_byte(uint8_t b);
static uint8_t i2c_bb_read_byte(bool sendAck);
static bool i2c_bb_write_reg(uint8_t addr7, uint8_t reg, uint8_t val);
static void i2c_bb_read_reg(uint8_t addr7, uint8_t reg, uint8_t cnt, uint8_t *buf);

static void i2cWrite(uint8_t reg, uint8_t val){
  if (useSoftI2C) {
    i2c_bb_write_reg(MPU_ADDR, reg, val);
    return;
  }
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
static void i2cRead(uint8_t reg, uint8_t cnt, uint8_t *buf){
  if (useSoftI2C) {
    i2c_bb_read_reg(MPU_ADDR, reg, cnt, buf);
    return;
  }
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, cnt);
  for (uint8_t i=0;i<cnt;i++) buf[i] = Wire.available() ? Wire.read() : 0;
}

static void i2cScanner() {
  Serial.println("I2C scanner (PB9=SDA, PB8=SCL)");
  bool found = false;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device @ 0x"); Serial.println(addr, HEX);
      if (addr == 0x68 || addr == 0x69) {
        MPU_ADDR = addr;
        found = true;
      }
    }
    delay(5);
  }
  imuAvailable = found;
  if (!found) Serial.println("MPU6050 not found at 0x68/0x69");
  else { Serial.print("Using MPU address: 0x"); Serial.println(MPU_ADDR, HEX); }
}

// Try to read WHO_AM_I register (0x75) from given address. Returns true if read.
static bool readWhoAmI(uint8_t addr, uint8_t &who) {
  who = 0;
  Wire.beginTransmission(addr);
  Wire.write(0x75);               // WHO_AM_I Register !!!
  if (Wire.endTransmission(false) != 0) return false; // device didn't ACK
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  who = Wire.read();
  return true;
}

// Diagnostic helper: print pin states and do a low-level WHO_AM_I trial
static void i2cDiagnostic(uint8_t addr) {
  Serial.print("I2C diag: SDA(PB9) = "); Serial.println(digitalRead(PB9));
  Serial.print("I2C diag: SCL(PB8) = "); Serial.println(digitalRead(PB8));

  Serial.print("BeginTransmission(0x"); Serial.print(addr, HEX); Serial.println(") -> trying register 0x75");
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  int r = Wire.endTransmission(false); // do not release bus
  Serial.print("endTransmission(false) returned: "); Serial.println(r);

  Serial.print("Requesting 1 byte from 0x"); Serial.println(addr, HEX);
  int got = Wire.requestFrom(addr, (uint8_t)1);
  Serial.print("requestFrom returned count= "); Serial.println(got);
  if (got == 1) {
    int b = Wire.read();
    Serial.print("WHO_AM_I raw = 0x");
    if (b < 16) Serial.print('0');
    Serial.println(b, HEX);
  } else {
    Serial.println("No bytes available from requestFrom()");
  }

  // Try a full endTransmission(true) variant too
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  int r2 = Wire.endTransmission(true);
  Serial.print("endTransmission(true) returned: "); Serial.println(r2);
}

// --- Soft I2C (bit-bang) probe on PB9 (SDA) / PB8 (SCL)
static void i2c_bb_delay() { delayMicroseconds(50); }
static void i2c_bb_start() {
  // SDA high, SCL high -> SDA low -> SCL low
  pinMode(PB9, INPUT_PULLUP); // SDA released (high)
  pinMode(PB8, INPUT_PULLUP); // SCL released (high)
  i2c_bb_delay();
  pinMode(PB9, OUTPUT); digitalWrite(PB9, LOW); // SDA low
  i2c_bb_delay();
  pinMode(PB8, OUTPUT); digitalWrite(PB8, LOW); // SCL low
  i2c_bb_delay();
}
static void i2c_bb_stop() {
  // SDA low, SCL low -> SCL high -> SDA high
  pinMode(PB9, OUTPUT); digitalWrite(PB9, LOW);
  pinMode(PB8, OUTPUT); digitalWrite(PB8, LOW);
  i2c_bb_delay();
  digitalWrite(PB8, HIGH); i2c_bb_delay(); // SCL high
  pinMode(PB9, INPUT_PULLUP); // SDA release -> goes high
  i2c_bb_delay();
}

// write a byte MSB first, return true if ACK received
static bool i2c_bb_write_byte(uint8_t b) {
  // Ensure SCL is driven low initially
  pinMode(PB8, OUTPUT); digitalWrite(PB8, LOW);
  for (int i = 7; i >= 0; --i) {
    bool bit = (b >> i) & 1;
    if (bit) {
      pinMode(PB9, INPUT_PULLUP); // release SDA -> high
    } else {
      pinMode(PB9, OUTPUT); digitalWrite(PB9, LOW);
    }
    i2c_bb_delay();
    // Clock high
    digitalWrite(PB8, HIGH);
    i2c_bb_delay();
    // Clock low
    digitalWrite(PB8, LOW);
    i2c_bb_delay();
  }
  // Release SDA for ACK
  pinMode(PB9, INPUT_PULLUP);
  i2c_bb_delay();
  // Clock pulse for ACK
  digitalWrite(PB8, HIGH); i2c_bb_delay();
  bool ack = (digitalRead(PB9) == LOW);
  digitalWrite(PB8, LOW); i2c_bb_delay();
  return ack;
}

// read a single byte (MSB first). If sendAck==true the master will pull SDA low
// during the ACK clock pulse, otherwise it will release SDA (NACK).
static uint8_t i2c_bb_read_byte(bool sendAck) {
  uint8_t val = 0;
  // Ensure SCL is output-driven
  pinMode(PB8, OUTPUT);
  // SDA released for input
  pinMode(PB9, INPUT_PULLUP);
  for (int i = 7; i >= 0; --i) {
    // Clock high
    digitalWrite(PB8, HIGH); i2c_bb_delay();
    int bit = digitalRead(PB9);
    if (bit) val |= (1 << i);
    // Clock low
    digitalWrite(PB8, LOW); i2c_bb_delay();
  }
  // ACK/NACK: if we should ACK, pull SDA low during the ACK clock pulse
  if (sendAck) {
    pinMode(PB9, OUTPUT); digitalWrite(PB9, LOW);
  } else {
    pinMode(PB9, INPUT_PULLUP); // release SDA -> NACK
  }
  // Pulse clock for ACK/NACK
  digitalWrite(PB8, HIGH); i2c_bb_delay();
  digitalWrite(PB8, LOW); i2c_bb_delay();
  // Release SDA after ACK/NACK
  pinMode(PB9, INPUT_PULLUP);
  return val;
}

// Perform a small probe: send address+W and address+R to check ACK, and
// attempt a WHO_AM_I read via bit-bang if ACK for write is present.
static int softI2CProbe(uint8_t addr7) {
  uint8_t addrW = (addr7 << 1) | 0;
  uint8_t addrR = (addr7 << 1) | 1;
  Serial.print("Soft-I2C probe @0x"); Serial.print(addr7, HEX);
  Serial.println(" (bit-bang)");

  // Ensure lines start released
  pinMode(PB9, INPUT_PULLUP); pinMode(PB8, INPUT_PULLUP);
  i2c_bb_delay();
  // START
  i2c_bb_start();
  bool ackW = i2c_bb_write_byte(addrW);
  Serial.print("Write address (W) ack: "); Serial.println(ackW ? "YES" : "NO");
  // If write ack, try to write register 0x75 and then restart+read
  if (ackW) {
    bool ackReg = i2c_bb_write_byte(0x75);
    Serial.print("Write reg 0x75 ack: "); Serial.println(ackReg ? "YES" : "NO");
    // Repeated START and switch to read
    i2c_bb_start();
    bool ackRaddr = i2c_bb_write_byte(addrR);
    Serial.print("Address (R) ack: "); Serial.println(ackRaddr ? "YES" : "NO");
    if (ackRaddr) {
      // single-byte read: master must NACK the last byte
      uint8_t who = i2c_bb_read_byte(false);
      Serial.print("Soft WHO_AM_I = 0x"); if (who < 16) Serial.print('0'); Serial.println(who, HEX);
      i2c_bb_stop();
      return who;
    }
  }
  i2c_bb_stop();
  return -1;
}

// Bit-bang helpers to read/write registers (multi-byte)
static bool i2c_bb_write_reg(uint8_t addr7, uint8_t reg, uint8_t val) {
  i2c_bb_start();
  if (!i2c_bb_write_byte((addr7 << 1) | 0)) { i2c_bb_stop(); return false; }
  if (!i2c_bb_write_byte(reg)) { i2c_bb_stop(); return false; }
  if (!i2c_bb_write_byte(val)) { i2c_bb_stop(); return false; }
  i2c_bb_stop();
  return true;
}

static void i2c_bb_read_reg(uint8_t addr7, uint8_t reg, uint8_t cnt, uint8_t *buf) {
  // Write register pointer
  i2c_bb_start();
  if (!i2c_bb_write_byte((addr7 << 1) | 0)) { i2c_bb_stop(); memset(buf, 0, cnt); return; }
  if (!i2c_bb_write_byte(reg)) { i2c_bb_stop(); memset(buf, 0, cnt); return; }
  // Repeated start and read
  i2c_bb_start();
  if (!i2c_bb_write_byte((addr7 << 1) | 1)) { i2c_bb_stop(); memset(buf, 0, cnt); return; }
    for (uint8_t i = 0; i < cnt; ++i) {
    // send ACK for all but the last byte
    bool sendAck = (i < (cnt - 1));
    buf[i] = i2c_bb_read_byte(sendAck);
  }
  i2c_bb_stop();
}

// Initialize I2C and the sensor; replace previous inline setup code
void sensor_init() {
  Serial.println("Starting I2C: Wire.begin(PB9, PB8)");
  Wire.begin(PB9, PB8);
  Wire.setClock(100000);
  Serial.println("I2C: using 100kHz for initial scan");
  i2cScanner();
  if (!imuAvailable) {
    Serial.println("Final attempt: switching to 400kHz and rescanning");
    Wire.setClock(400000);
    i2cScanner();
  }

  if (!imuAvailable) {
    uint8_t who;
    if (readWhoAmI(0x68, who)) {
      Serial.print("WHO_AM_I @0x68 = 0x"); Serial.println(who, HEX);
      MPU_ADDR = 0x68; imuAvailable = true;
    } else if (readWhoAmI(0x69, who)) {
      Serial.print("WHO_AM_I @0x69 = 0x"); Serial.println(who, HEX);
      MPU_ADDR = 0x69; imuAvailable = true;
    } else {
      Serial.println("WHO_AM_I read failed for 0x68/0x69");
      Serial.println("Running detailed I2C diagnostic for 0x68 and 0x69");
      i2cDiagnostic(0x68);
      i2cDiagnostic(0x69);
      Serial.println("Running soft (bit-bang) I2C probe for 0x68 and 0x69");
      int who = softI2CProbe(0x68);
      if (who >= 0) {
        Serial.print("Soft probe found device at 0x68 WHO_AM_I=0x"); Serial.println(who, HEX);
        MPU_ADDR = 0x68; imuAvailable = true; useSoftI2C = true;
      } else {
        who = softI2CProbe(0x69);
        if (who >= 0) {
          Serial.print("Soft probe found device at 0x69 WHO_AM_I=0x"); Serial.println(who, HEX);
          MPU_ADDR = 0x69; imuAvailable = true; useSoftI2C = true;
        } else {
          Serial.println("Soft probe found no device at 0x68/0x69");
        }
      }
    }
  }

  if (imuAvailable) {
    // wake up sensor and calibrate gyros
    i2cWrite(PWR_MGMT_1, 0x00);
    delay(100);
    // Ensure accelerometer is set to +/-2g for best angle resolution
    i2cWrite(ACCEL_CONFIG, 0x00); // AFS_SEL = 0 -> +/-2g
    delay(10);
    // Read back ACCEL_CONFIG and set accScale accordingly
    uint8_t aconf = 0;
    i2cRead(ACCEL_CONFIG, 1, &aconf);
    uint8_t afs = (aconf >> 3) & 0x03;
    switch (afs) {
      case 0: accScale = 16384.0f; break; // +/-2g
      case 1: accScale = 8192.0f; break;  // +/-4g
      case 2: accScale = 4096.0f; break;  // +/-8g
      case 3: accScale = 2048.0f; break;  // +/-16g
      default: accScale = 16384.0f; break;
    }
    Serial.print("ACCEL_CONFIG read: 0x"); Serial.print(aconf, HEX);
    Serial.print(" -> accScale = "); Serial.println(accScale);
    const int cal = 500; // increase calibration samples for more stable gyro offsets
    long gxSum = 0, gySum = 0, gzSum = 0;
    for (int i = 0; i < cal; ++i) {
      uint8_t b[6];
      i2cRead(GYRO_XOUT_H, 6, b);
      int16_t gx = (b[0] << 8) | b[1];
      int16_t gy = (b[2] << 8) | b[3];
      int16_t gz = (b[4] << 8) | b[5];
      gxSum += gx; gySum += gy; gzSum += gz;
      delay(5);
    }
    gyroXoff = gxSum / (float)cal;
    gyroYoff = gySum / (float)cal;
    gyroZoff = gzSum / (float)cal;
    Serial.print("Gyro offsets: "); Serial.print(gyroXoff); Serial.print(", "); Serial.println(gyroYoff);
    // Initialize fused angles from the accelerometer to avoid large gyro integration bias
    {
      uint8_t b[14];
      i2cRead(ACCEL_XOUT_H, 14, b);
      int16_t ax = (b[0] << 8) | b[1];
      int16_t ay = (b[2] << 8) | b[3];
      int16_t az = (b[4] << 8) | b[5];
      const float accScaleLocal = 16384.0f;
      float axg = ax / accScaleLocal, ayg = ay / accScaleLocal, azg = az / accScaleLocal;
      float initRoll  = atan2(ayg, azg) * 180.0f / M_PI;
      float initPitch = atan2(-axg, sqrt(ayg*ayg + azg*azg)) * 180.0f / M_PI;
      angleRoll = initRoll;
      anglePitch = initPitch;
      Serial.print("Init angles from accel: Roll="); Serial.print(angleRoll,2);
      Serial.print(", Pitch="); Serial.println(anglePitch,2);
    }
    imuLastMs = millis();
  }
}

// sensor_loop: read IMU and update fused angles + measuredRoll/Pitch
void sensor_loop() {
  if (!imuAvailable) return;
  unsigned long now = millis();
  float dt = (now - imuLastMs) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  imuLastMs = now;
  uint8_t buf[14];
  i2cRead(ACCEL_XOUT_H, 14, buf);
  int16_t ax = (buf[0] << 8) | buf[1];
  int16_t ay = (buf[2] << 8) | buf[3];
  int16_t az = (buf[4] << 8) | buf[5];
  int16_t gx = (buf[8] << 8) | buf[9];
  int16_t gy = (buf[10] << 8) | buf[11];
  const float gyroScale = 131.0f;  // +/-250deg/s
  float axg = ax / accScale, ayg = ay / accScale, azg = az / accScale;
  float gxdps = (gx - gyroXoff) / gyroScale;
  float gydps = (gy - gyroYoff) / gyroScale;
  float rollAcc  = atan2(ayg, azg) * 180.0f / M_PI;
  float pitchAcc = atan2(-axg, sqrt(ayg*ayg + azg*azg)) * 180.0f / M_PI;
  // integrate gyro
  angleRoll  += gxdps * dt;
  anglePitch += gydps * dt;
  float accMag = sqrt(axg*axg + ayg*ayg + azg*azg);
  if (accMag >= 0.5f && accMag <= 1.5f) {
    angleRoll  = alpha * angleRoll  + (1.0f - alpha) * rollAcc;
    anglePitch = alpha * anglePitch + (1.0f - alpha) * pitchAcc;
  }
  // Map to -100..100 and store atomically
  int r = (int)round(angleRoll);
  int p = (int)round(anglePitch);
  if (r < -100) r = -100; if (r > 100) r = 100;
  if (p < -100) p = -100; if (p > 100) p = 100;
  noInterrupts();
  measuredRoll = r;
  measuredPitch = p;
  interrupts();
}
