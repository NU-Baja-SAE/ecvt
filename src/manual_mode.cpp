#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>
#include "pulseCounter.h"

#define IDLE_RPM 1900
#define IDLE_SHEAVE_SETPOINT -90

unsigned short pot_position;
enum MODE_T {
    POWER_MODE,
    TORQUE_MODE,
    POSITION_1,
    POSITION_2,
    POSITION_3,
    POSITION_4
};
// Mode read will read the value from the mode control and update the mode as well as update the target 
// TODO: should this function return a void, or be void and update the global state?
void mode_read()
{
    // 
    // read the potentiometer 
    pot_position = 1;
    //____UNcommment this in the future_____

    //pot_position = analogRead(MODE_PIN)    

    // Set enum mode variable to be the mode associated with the pot position
    //
    if (0 < pot_position < 8192)
        MODE_T::POWER_MODE;
    else if (8192 < pot_position < 16384)
        MODE_T::TORQUE_MODE;
    else if (8192 < pot_position < 24576)
        MODE_T::POSITION_1;
    else if (24576 < pot_position < 32768)
        MODE_T::POSITION_2;
    else if (32768 < pot_position < 40960)
        MODE_T::POSITION_3;
    else if (pot_position > 40960)
        MODE_T::POSITION_4;  
}


