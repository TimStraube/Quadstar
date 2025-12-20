/*
 * controller.cpp
 * Arduino-friendly minimal mixer: computes 4 motor PWM duty values from
 * roll/pitch/thrust percentages and writes them with analogWrite to
 * configured pins.
 */

#include "controller.h"
#include "stm32f4xx.h" // provide TIM1 definition when building for STM32
#include "stm32f4xx_hal.h" // for HAL_RCC_GetPCLK2Freq()
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

        // Minimal TIM1 PWM setup for 50 Hz frame (20 ms)
        // We'll configure timer tick = 1 MHz (1 µs per tick) when possible,
        // so ARR = 20000 -> 20 ms period.
        TIM1->CR1 = 0; // stop timer
        // Determine timer clock (PCLK2, timers double when APB prescaler != 1)
        uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
        // If APB2 prescaler not 1, timer clock is doubled
        uint32_t tim_clk = pclk2;
        uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> 11;
        if (ppre2 != 0) {
            tim_clk = pclk2 * 2;
        }
        // compute prescaler to get 1 MHz timer tick (1 µs)
        uint32_t desired_tick_hz = 1000000UL;
        uint32_t psc = (tim_clk / desired_tick_hz);
        if (psc == 0) psc = 1;
        TIM1->PSC = (uint16_t)(psc - 1);
        // Set ARR for 20 ms (20000 µs)
        TIM1->ARR = 20000 - 1;
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
void control(const double motor_pwm[4], float setpoint_thrust, float skalar, float pwmLower, float pwmUpper)
{
    // clamp setpoint_thrust
    if (setpoint_thrust < 0.0f) setpoint_thrust = 0.0f;
    if (setpoint_thrust > 1.0f) setpoint_thrust = 1.0f;

    // compute averaged command per motor (smoothing)
    // motor_pwm[] is expected in [0..1]. Previously the code divided by 3
    // which dramatically reduced the commanded pulse widths. Replace that
    // with an exponential smoothing filter so the controller remains
    // responsive but avoids large transients.
    double avg[4];
    static double prev_avg[4] = {0.0, 0.0, 0.0, 0.0};
    const double alpha = 0.6; // smoothing factor: 0..1 (1 = no smoothing)
    for (int i = 0; i < 4; ++i) {
        double v = motor_pwm[i];
        if (v < 0.0) v = 0.0; if (v > 1.0) v = 1.0;
        avg[i] = alpha * v + (1.0 - alpha) * prev_avg[i];
        prev_avg[i] = avg[i];
    }

    // Interpret avg[] as normalized motor share in [0..1]. Map 0 -> pwmLower (µs), 1 -> pwmUpper (µs).
    // With timer tick set to 1 µs, CCR value equals pulse width in µs.
    int span = (int)roundf(pwmUpper - pwmLower); // in µs

    // ---------- ESC INIT STATE MACHINE ----------
    // Sequence: 1000 µs for 5s -> 1500 µs for 0.5s -> ARMED (normal control)
    enum EscState {
        ESC_INIT_LOW,
        ESC_INIT_MID,
        ESC_ARMED
    };

    static EscState escState = ESC_INIT_LOW;
    static unsigned long escStateMs = 0;
    unsigned long now = millis();
    if (escStateMs == 0) escStateMs = now;

    // Compute pulse widths (µs) per motor from mixer output
    int pulse_us[4];
    for (int i = 0; i < 4; ++i) {
        float v = (float)avg[i];
        if (v < 0.0f) v = 0.0f; if (v > 1.0f) v = 1.0f;
        pulse_us[i] = (int)roundf(pwmLower + v * span);
    }

    // If still in init states, override pulses accordingly
    switch (escState) {
        case ESC_INIT_LOW:
            // 1000 µs for 5 seconds
            for (int i = 0; i < 4; i++) pulse_us[i] = (int)pwmLower;
            if (now - escStateMs >= 5000UL) {
                escState = ESC_INIT_MID;
                escStateMs = now;
            }
            break;
        case ESC_INIT_MID:
            // 1500 µs (Neutral / Mid Throttle)
            for (int i = 0; i < 4; i++) pulse_us[i] = 1500;
            if (now - escStateMs >= 500UL) {
                escState = ESC_ARMED;
                escStateMs = now;
            }
            break;
        case ESC_ARMED:
            // normal operation — already computed pulse_us
            break;
    }

    // Clamp pulses to timer range and write directly to CCR (1 µs ticks)
    uint32_t arr = TIM1->ARR ? TIM1->ARR : 20000;
    for (int i = 0; i < 4; ++i) {
        if (pulse_us[i] < 0) pulse_us[i] = 0;
        if ((uint32_t)pulse_us[i] > arr) pulse_us[i] = (int)arr;
    }

    TIM1->CCR1 = (uint32_t)pulse_us[0];
    TIM1->CCR2 = (uint32_t)pulse_us[1];
    TIM1->CCR3 = (uint32_t)pulse_us[2];
    TIM1->CCR4 = (uint32_t)pulse_us[3];
    // Optionally: print duties as percent for observation
    static unsigned long lastPrintMs = 0;
    const unsigned long printIntervalMs = 100;
    unsigned long nowPrint = millis();
    if ((nowPrint - lastPrintMs) >= printIntervalMs) {
        lastPrintMs = nowPrint;
    // compute percent of frame (20000 µs => 100%)
    int p0 = (int)round(((float)TIM1->CCR1 / 20000.0f) * 100.0f);
    int p1 = (int)round(((float)TIM1->CCR2 / 20000.0f) * 100.0f);
    int p2 = (int)round(((float)TIM1->CCR3 / 20000.0f) * 100.0f);
    int p3 = (int)round(((float)TIM1->CCR4 / 20000.0f) * 100.0f);
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

