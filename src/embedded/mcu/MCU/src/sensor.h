/* sensor.h - IMU (MPU6050) helper interface */
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

// sensor state exported to main
extern bool imuAvailable;
extern unsigned long imuLastMs;
extern float angleRoll;
extern float anglePitch;
extern volatile int measuredRoll;
extern volatile int measuredPitch;

// Initialize I2C and the IMU (call from setup)
void sensor_init();

// Poll/update sensor (call from loop frequently)
void sensor_loop();

#endif // SENSOR_H
