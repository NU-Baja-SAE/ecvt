#include "wheel_rpm.h"
#include "pins.h"

volatile unsigned long wheel_pulse_count = 0;
unsigned long last_wheel_rpm_calculation_time = 0;

void IRAM_ATTR wheel_hall_interrupt() {
    wheel_pulse_count++;
}

void setup_wheel_rpm() {
    pinMode(WHEEL_HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(WHEEL_HALL_PIN), wheel_hall_interrupt, RISING);
    wheel_pulse_count = 0;
    last_wheel_rpm_calculation_time = millis();
    Serial.println("Wheel RPM sensor initialized");
}

float get_wheel_rpm() {
    unsigned long current_time = millis();
    unsigned long time_elapsed = current_time - last_wheel_rpm_calculation_time;

    if (time_elapsed >= WHEEL_RPM_CALCULATION_INTERVAL_MS) {
        float rpm = 0.0;

        if (time_elapsed > 0) {
            rpm = (wheel_pulse_count * 60000.0) / (time_elapsed * WHEEL_MAGNETS_PER_REVOLUTION);
        }

        wheel_pulse_count = 0;
        last_wheel_rpm_calculation_time = current_time;

        return rpm;
    }

    return -1;
}

void reset_wheel_rpm_counter() {
    wheel_pulse_count = 0;
    last_wheel_rpm_calculation_time = millis();
}