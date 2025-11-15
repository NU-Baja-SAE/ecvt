#include <Arduino.h>
#include "motor.h"
#include "pins.h"

// #define POS_Kp 3.8 
#define POS_Kp 1.5 //4.3 
#define POS_Ki 0.042
#define POS_Kd 30.0  //80
#define POS_MAX_I_TERM 3000.0
#define Kp_multiplier 0.3 // use a lower Kp in the negative direction


#define RPM_Kp 0.002
#define RPM_Kd 0




void setup_pid_task();