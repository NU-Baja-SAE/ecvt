#ifndef WHEEL_RPM_H
#define WHEEL_RPM_H

#include <Arduino.h>

#define WHEEL_HALL_PIN 25
#define MAGNETS_PER_REVOLUTION 6
#define RPM_CALCULATION_INTERVAL_MS 200

void setup_wheel_rpm();
float get_wheel_rpm();
void reset_wheel_rpm_counter();

extern volatile unsigned long wheel_pulse_count;
extern unsigned long last_rpm_calculation_time;

#endif