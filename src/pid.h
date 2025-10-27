#include <Arduino.h>
#include "motor.h"
#include "pins.h"

// #define POS_Kp 3.8 
#define POS_Kp 10 //4.3 
#define POS_Ki 0
#define POS_Kd 5  //80
#define POS_MAX_I_TERM 50


#define RPM_Kp 0.002
#define RPM_Kd 0




void setup_pid_task();