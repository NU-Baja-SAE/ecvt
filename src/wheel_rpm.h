#ifndef WHEEL_RPM_H
#define WHEEL_RPM_H

#include <Arduino.h>

#define WHEEL_MAGNETS_PER_REVOLUTION 4
#define WHEEL_RPM_CALCULATION_INTERVAL_MS 500

void setup_wheel_rpm();
float get_wheel_rpm();
void reset_wheel_rpm_counter();

extern volatile unsigned long wheel_pulse_count;
extern unsigned long last_wheel_rpm_calculation_time;

#endif