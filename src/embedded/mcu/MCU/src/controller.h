/*
 * controller.h
 * Simple quad controller helper: computes motor PWMs from motor commands and
 * applies them to TIM1 CCR registers.
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>

// Arduino-friendly minimal controller API
// Configure which Arduino pins are used for the 4 motor PWM outputs.
void controllerConfigurePins(int m1_pin, int m2_pin, int m3_pin, int m4_pin);

// Update motor outputs from attitude/thrust inputs
// roll, pitch: in integer percent (-100..100)
// thrust: in integer percent (0..100)
void controllerSetFromAttitude(int roll_percent, int pitch_percent, int thrust_percent);

// Executes a control step: computes motor PWM duties from motor command history
// and applies them to TIM1 CCR registers.
void control(const double motor_pwm[4], float setpoint_thrust, float skalar, float pwmLower, float pwmUpper);

#endif // CONTROLLER_H
