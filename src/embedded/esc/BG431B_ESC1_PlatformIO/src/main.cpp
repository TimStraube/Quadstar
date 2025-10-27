#include <Arduino.h>
#include "SimpleFOC.h"

// Motor und Treiber initialisieren
BLDCMotor motor = BLDCMotor(12);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Commander-Instanz
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.motion(&motor, cmd); }

// --- PWM Input globals ---
const uint8_t PWM_INPUT_PIN = PA15;
volatile uint32_t lastRise = 0;
volatile uint32_t pulseWidth = 1500; // default safe value

void pwmChange() {
    if (digitalRead(PWM_INPUT_PIN)) {
        lastRise = micros(); // RISING
    } else {
        pulseWidth = micros() - lastRise; // FALLING
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);

    // Treiber initialisieren
    driver.voltage_power_supply = 22;
    driver.init();
    motor.linkDriver(&driver);

    // Stromsensor initialisieren
    currentSense.linkDriver(&driver);
    currentSense.init();
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    // Motorparameter setzen
    motor.voltage_sensor_align = 1;
    motor.velocity_index_search = 1;
    motor.voltage_limit = 6;
    motor.velocity_limit = 6000; // 6000 RPM

    motor.controller = MotionControlType::velocity_openloop;
    motor.torque_controller = TorqueControlType::foc_current;

    motor.PID_current_q.P = motor.PID_current_d.P = 0.1;
    motor.PID_current_q.I = motor.PID_current_d.I = 10;

    motor.PID_velocity.P = 0.5;
    motor.PID_velocity.I = 1;
    motor.PID_velocity.output_ramp = 1000;
    motor.LPF_velocity.Tf = 0.01;

    motor.P_angle.P = 10;

    // Serielle Kommunikation initialisieren
    motor.useMonitoring(Serial);

    // Motor initialisieren
    motor.init();
    if (motor.initFOC() == 0) { // Sensorlose Initialisierung
        Serial.println(F("FOC initialisiert"));
    } else {
        Serial.println(F("FOC Initialisierung fehlgeschlagen"));
    }
    motor.initFOC();
    motor.target = 0; 

    // Befehl hinzufügen
    command.add('T', doTarget, "target velocity");

    // PWM Input einrichten (PA15 Interrupt)
    pinMode(PWM_INPUT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmChange, CHANGE);

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target velocity using serial terminal:"));
    delay(1000);
}

// --- Filter + deadband globals ---
float filteredPWM = 1500;
int lastTarget = 150;
float userTarget = 150; // Initialwert für Ramp-up

void loop()
{
    motor.move();
    motor.loopFOC();
    command.run();

    static bool rampDone = false;
    static unsigned long lastUpdate = 0;

    // Ramp-up bis 150 beim Start
    if (!rampDone) {
        if (millis() - lastUpdate >= 100) {
            if (motor.target < 150) {
                motor.target += 2;
                Serial.print("Startup ramp, target: ");
                Serial.println(motor.target);
            } else {
                rampDone = true;
            }
            lastUpdate = millis();
        }
        return; // PWM erst nach Ramp-up auswerten
    }

    // --- PWM-Eingang mit pulseIn ---
    uint32_t pwmRaw = pulseIn(PWM_INPUT_PIN, HIGH, 5000); // Timeout 5ms
    if (pwmRaw > 0) {
        filteredPWM = 0.9f * filteredPWM + 0.1f * pwmRaw;
        int newTarget = map(filteredPWM, 1000, 2000, 0, 500);
        userTarget = newTarget;
        Serial.print("PWM input (filtered): ");
        Serial.print(filteredPWM, 1);
        Serial.print(" µs -> userTarget: ");
        Serial.println(userTarget);
    } else {
        userTarget = 0;
    }

    // motor.target nähert sich userTarget with Ramp
    if (millis() - lastUpdate >= 100) {
        if (motor.target < userTarget) {
            motor.target += 2;
            if (motor.target > userTarget) motor.target = userTarget;
        } else if (motor.target > userTarget) {
            motor.target -= 2;
            if (motor.target < userTarget) motor.target = userTarget;
        }
        Serial.print("Ramp to userTarget, motor.target: ");
        Serial.println(motor.target);
        lastUpdate = millis();
    }
}
