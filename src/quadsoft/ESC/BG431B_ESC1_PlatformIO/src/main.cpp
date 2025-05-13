#include <Arduino.h>
#include "SimpleFOC.h"

// Motor und Treiber initialisieren
BLDCMotor motor = BLDCMotor(12);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Commander-Instanz
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.motion(&motor, cmd); }

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
    motor.velocity_index_search = 3;
    motor.voltage_limit = 6;
    motor.velocity_limit = 1000;

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

    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target velocity using serial terminal:"));
    delay(1000);
}

void loop() {
    motor.move();
    motor.loopFOC();
    command.run();

    // Überprüfen, ob serielle Daten verfügbar sind
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Entfernt führende und nachfolgende Leerzeichen, einschließlich \r
        if (input.length() > 0 && input != "\r") {
            float targetSpeed = input.toFloat();
            motor.target = targetSpeed;
            Serial.print("Neue Zielgeschwindigkeit: ");
            Serial.println(motor.target);
        }
    }

    // Entfernen der automatischen Geschwindigkeitssteigerung
    // static unsigned long lastUpdate = 0;
    // if (millis() - lastUpdate >= 100 && motor.target < 50) {
    //     motor.target += 1;
    //     Serial.print("Speed increased, new target: ");
    //     Serial.println(motor.target);
    //     lastUpdate = millis();
    // }
}