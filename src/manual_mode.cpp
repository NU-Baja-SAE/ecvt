#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>
#include "pulseCounter.h"
#include "manual_mode.h"


// A function that reads the manual mode dial and assigns a state based on the dial position
Mode_T mode_read() {
    int dial_position = digitalRead(MANUAL_MODE_PIN); // read the mode the car is in 
    if (dial_position == LOW) {
        return POWER_MODE;
    }
    else if (dial_position == HIGH){
        return TORQUE_MODE;
    }
    return POWER_MODE; //Should not reach here

}
