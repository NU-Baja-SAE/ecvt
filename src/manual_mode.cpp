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
    int dial_position = digitalRead(MANUAL_MODE_PIN); // read the mode the car is in 
    if (dial_position == 0) {
        car_mode = POWER_MODE;
    }
    else if (dial_position == 0){
        car_mode = TORQUE_MODE;
    }

}
