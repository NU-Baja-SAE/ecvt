#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>
#include "pulseCounter.h"

#define IDLE_RPM 1900
#define IDLE_SHEAVE_SETPOINT -90

int mode;
bool manual_mode; // this boolean will be refrenced in PID file when swutching between manual controll
int target_setpoint;  // the setpoints for the sheaves in manula mode
float setpoint_error; // the error between the target setpoint and the current 
float motor_speed; // the value to set the motor speed based on the setpoint 

// Mode read will read the value from the mode control and update the mode as well as update the target 
// TODO: should this function return a bool, or be void and update the global state?
bool mode_read()
{
    // 
    // read the mode value
    // Set mode variable to be the value of the dial
    //
    if (mode <= 3) {
        manual_mode = true;  
        target_setpoint = 1000; // use math to relate mode read value and target setpoints of branch statements 
    }
    else {
        manual_mode = false;  // Update the value of the boole
    }
}

//Will run manual control task loop instead of PID loop task when in manual control mode 
void move_to_teraget_setpoint()
{
    int pos = read_pos(); // position of the sheaves
    if (get_engine_rpm() < IDLE_RPM) // if the rpm is less than the idle rpm
    {
        setpoint_error = IDLE_SHEAVE_SETPOINT - pos;
        motor_speed = setpoint_error * 1234;  // calculate the motor speed to move the sheaves  
        set_direction_speed((int)motor_speed);
    }
    else {
        setpoint_error = target_setpoint - pos;
        motor_speed = setpoint_error * 1234;  // calculate the motor speed to move the sheaves  
        set_direction_speed((int)motor_speed);
    }
}

// PID loop will call get mode rpm instead of target rpm when runnning 
int get_mode_rpm() {
    if (mode == 0) {             // if the engine is in power mode return 3000
        return 3000;
    }
    else {            // else returnn 2400
        return 2400;
    }
}
