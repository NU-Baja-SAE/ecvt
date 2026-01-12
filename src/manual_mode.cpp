#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>
#include "pulseCounter.h"

// Define the differnet possible state of the car with an enum
enum Mode_T{
    power_mode,
    torque_mode,
    position_one,
    position_two,
    position_three,
    position_four
};

// A function that reads the manual mode dial and assignes an state based on the dial posiont
void mode_read()
{
    int dial_position = analogRead(POT_PIN); // read the mode the car is in 
    if (10892 > dial_position > 21785) {
        enum Mode_T car_mode = power_mode;
    }
    else if (21758 < dial_position < 32677)
    {
        enum Mode_T car_mode = torque_mode;
    }
    else if (32677 < dial_position < 43569)
    {
        enum Mode_T car_mode = position_one;
    }
    else if (43569 < dial_position < 54461) {
        enum Mode_T car_mode = position_two;
    }
    else if (54461 < dial_position < 65355) {
        enum Mode_T car_mode = position_three;
    }
    else if (0 < dial_position < 10892) {
        enum Mode_T car_mode = position_four;
    }
}
