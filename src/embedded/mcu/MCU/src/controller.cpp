
/*
 * controller.cpp
 * Arduino-friendly minimal mixer: computes 4 motor PWM duty values from
 * roll/pitch/thrust percentages and writes them with analogWrite to
 * configured pins.
 */

#include "controller.h"
#include "stm32f4xx.h" // provide TIM1 definition when building for STM32
#include <Arduino.h>

static int motorPins[4] = {-1, -1, -1, -1};
static bool pinsConfigured = false;

void controllerConfigurePins(int m1_pin, int m2_pin, int m3_pin, int m4_pin) {
    motorPins[0] = m1_pin;
    motorPins[1] = m2_pin;
    motorPins[2] = m3_pin;
    motorPins[3] = m4_pin;
    for (int i = 0; i < 4; ++i) {
        if (motorPins[i] >= 0) pinMode(motorPins[i], OUTPUT);
    }
    pinsConfigured = true;

    // If the pins are PA8..PA11 (TIM1 CH1..CH4) initialize TIM1 for PWM
    // so that direct writes to TIM1->CCRn in control have an effect.
    // This is a minimal, conservative setup: ARR is set to 30259 to match
    // the scaling used elsewhere in the project.
    bool useTIM1 = (motorPins[0] == PA8) && (motorPins[1] == PA9) && (motorPins[2] == PA10) && (motorPins[3] == PA11);
    if (useTIM1) {
        // Enable GPIOA clock (for PA8..PA11) and TIM1 clock
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

        // Configure PA8..PA11 to AF mode (AF1 = TIM1)
        // MODER: 10 = Alternate function
        for (int pin = 8; pin <= 11; ++pin) {
            uint32_t pos = pin * 2;
            // clear then set to 10
            GPIOA->MODER &= ~(0x3u << pos);
            GPIOA->MODER |=  (0x2u << pos);
        }
        // AFRH: each pin uses 4 bits in AFRH for pins 8..15
        // Set AF1 (value 0x1) for pins 8..11
        for (int pin = 8; pin <= 11; ++pin) {
            int idx = pin - 8;
            GPIOA->AFR[1] &= ~(0xFu << (idx * 4));
            GPIOA->AFR[1] |=  (0x1u << (idx * 4)); // AF1
        }

        // Minimal TIM1 PWM setup
        // Stop timer
        TIM1->CR1 = 0;
        // No prescaler for now
        TIM1->PSC = 0;
        // Set auto-reload to match original scaling constant
        TIM1->ARR = 30259;
        // Reset control registers related to capture/compare
        TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0; TIM1->CCR4 = 0;

        // Configure channels as PWM mode 1 and enable preload
        // CC1
        TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE | TIM_CCMR1_CC1S);
        TIM1->CCMR1 |= (6 << 4); // OC1M = 110 -> PWM mode 1
        TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
        // CC2
        TIM1->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE | TIM_CCMR1_CC2S);
        TIM1->CCMR1 |= (6 << 12); // OC2M = 110
        TIM1->CCMR1 |= TIM_CCMR1_OC2PE;
        // CC3
        TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE | TIM_CCMR2_CC3S);
        TIM1->CCMR2 |= (6 << 4); // OC3M = 110
        TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
        // CC4
        TIM1->CCMR2 &= ~(TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE | TIM_CCMR2_CC4S);
        TIM1->CCMR2 |= (6 << 12); // OC4M = 110
        TIM1->CCMR2 |= TIM_CCMR2_OC4PE;

        // Enable capture/compare outputs (active high)
        TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

        // Main output enable (MOE) in BDTR register is required for advanced-control timers
        TIM1->BDTR |= TIM_BDTR_MOE;

        // Enable ARR preload
        TIM1->CR1 |= TIM_CR1_ARPE;
        // Start timer counter
        TIM1->CR1 |= TIM_CR1_CEN;
    }
}

// Low-level apply that mirrors the original HAL TIM1 CCR writes from your code
void control(const double motor_pwm[4], const double motor_pwm_tminus1[4], const double motor_pwm_tminus2[4], float setpoint_thrust, float skalar, float pwmLower, float pwmUpper)
{
    // clamp setpoint_thrust
    if (setpoint_thrust < 0.0f) setpoint_thrust = 0.0f;
    if (setpoint_thrust > 1.0f) setpoint_thrust = 1.0f;

    // compute averaged command per motor (smoothing)
    double avg[4];
    for (int i = 0; i < 4; ++i) {
        avg[i] = (motor_pwm[i] + motor_pwm_tminus1[i] + motor_pwm_tminus2[i]) / 3.0;
    }

    // Interpret avg[] as normalized motor share in [0..1]. Map 0 -> pwmLower, 1 -> pwmUpper.
    // CCR = (((pwmUpper - pwmLower) * avg + pwmLower) / 1000.0f) * 30259
    float span = pwmUpper - pwmLower;
    uint32_t ccr0 = (uint32_t)((((span * (float)avg[0]) + pwmLower) / 1000.0f) * 30259.0f);
    uint32_t ccr1 = (uint32_t)((((span * (float)avg[1]) + pwmLower) / 1000.0f) * 30259.0f);
    uint32_t ccr2 = (uint32_t)((((span * (float)avg[2]) + pwmLower) / 1000.0f) * 30259.0f);
    uint32_t ccr3 = (uint32_t)((((span * (float)avg[3]) + pwmLower) / 1000.0f) * 30259.0f);

    TIM1->CCR1 = ccr0;
    TIM1->CCR2 = ccr1;
    TIM1->CCR3 = ccr2;
    TIM1->CCR4 = ccr3;

    // Optionally: print duties as percent for observation
    static unsigned long lastPrintMs = 0;
    const unsigned long printIntervalMs = 100;
    unsigned long now = millis();
    if ((now - lastPrintMs) >= printIntervalMs) {
        lastPrintMs = now;
        // compute percent from CCR (30259 => 100%)
        int p0 = (int)round((TIM1->CCR1 / 30259.0f) * 100.0f);
        int p1 = (int)round((TIM1->CCR2 / 30259.0f) * 100.0f);
        int p2 = (int)round((TIM1->CCR3 / 30259.0f) * 100.0f);
        int p3 = (int)round((TIM1->CCR4 / 30259.0f) * 100.0f);
        Serial.print("MOT%: ");
        Serial.print(p0); Serial.print(",");
        Serial.print(p1); Serial.print(",");
        Serial.print(p2); Serial.print(",");
        Serial.println(p3);
    }
}

// Simple mixer: maps roll/pitch [-100..100] and thrust [0..100] to per-motor
// duty (0..255). Mixer uses + configuration:
// m1 = thrust + pitch + roll
// m2 = thrust + pitch - roll
// m3 = thrust - pitch - roll
// m4 = thrust - pitch + roll
// Gains are conservative to avoid saturating motors when roll/pitch max.
void controllerSetFromAttitude(int roll_percent, int pitch_percent, int thrust_percent) {
    if (!pinsConfigured) return; // no-op if pins not set

    // clamp
    if (roll_percent < -100) roll_percent = -100; if (roll_percent > 100) roll_percent = 100;
    if (pitch_percent < -100) pitch_percent = -100; if (pitch_percent > 100) pitch_percent = 100;
    if (thrust_percent < 0) thrust_percent = 0; if (thrust_percent > 100) thrust_percent = 100;

    // Normalize to 0..1
    float t = thrust_percent / 100.0f;
    float r = roll_percent / 100.0f;
    float p = pitch_percent / 100.0f;

    // Conservative attitude gain to avoid large transient torques
    const float attitude_gain = 0.4f; // how much roll/pitch influences motor thrust

    // Compute per-motor (unnormalized) thrust values
    float m[4];
    m[0] = t + attitude_gain * ( p + r);
    m[1] = t + attitude_gain * ( p - r);
    m[2] = t + attitude_gain * (-p - r);
    m[3] = t + attitude_gain * (-p + r);

    // Find min/max to renormalize into [0,1] if necessary
    float minv = m[0], maxv = m[0];
    for (int i = 1; i < 4; ++i) {
        if (m[i] < minv) minv = m[i];
        if (m[i] > maxv) maxv = m[i];
    }

    // If values exceed [0,1], scale them back while preserving relative differences
    if (minv < 0.0f || maxv > 1.0f) {
        float span = maxv - minv;
        if (span <= 0.0f) span = 1.0f;
        for (int i = 0; i < 4; ++i) {
            m[i] = (m[i] - minv) / span; // now in 0..1
        }
    }

    // Map 0..1 to analogWrite (0..255)
    for (int i = 0; i < 4; ++i) {
        int duty = (int)round(constrain(m[i], 0.0f, 1.0f) * 255.0f);
        analogWrite(motorPins[i], duty);
    }
    // Print motor duties in percent (0..100%) at limited rate to avoid flooding Serial
    static unsigned long lastPrintMs = 0;
    const unsigned long printIntervalMs = 100; // print every 100 ms
    unsigned long now = millis();
    if ((now - lastPrintMs) >= printIntervalMs) {
        lastPrintMs = now;
        int pct[4];
        for (int i = 0; i < 4; ++i) {
            int duty = (int)round(constrain(m[i], 0.0f, 1.0f) * 255.0f);
            pct[i] = (int)round((duty / 255.0f) * 100.0f);
        }
        Serial.print("MOT%: ");
        Serial.print(pct[0]); Serial.print(",");
        Serial.print(pct[1]); Serial.print(",");
        Serial.print(pct[2]); Serial.print(",");
        Serial.println(pct[3]);
    }
}

