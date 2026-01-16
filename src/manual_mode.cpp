#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>
#include "pulseCounter.h"
#include "manual_mode.h"


// A function that reads the manual mode dial and assignes a state based on the dial posiont
void mode_read() {
    int dial_position = analogRead(POT_PIN); // read the mode the car is in 
    if (10892 > dial_position > 21785) {
        car_mode = power_mode;
    }
    else if (21758 < dial_position < 32677){
        car_mode = torque_mode;
    }
    else if (32677 < dial_position < 43569){
        car_mode = position_one;
    }
    else if (43569 < dial_position < 54461) {
        car_mode = position_two;
    }
    else if (54461 < dial_position < 65355) {
        car_mode = position_three;
    }
    else if (0 < dial_position < 10892) {
        car_mode = position_four;
    }
}
