#include "FreeRTOSConfig.h"

#define POT_PIN 32 // measures at potentiometer slider
#define PWM_PIN 13 // SV on driver, sends pulses to the motor 
#define DIRECTION_PIN 26 // F/R on driver, controls the motor's direction
#define PRIMARY_HALL_PIN 33 // reads each time the hall sensor passes a magnet
#define SECONDARY_HALL_PIN 23 // reads each time the hall sensor passes a magnet
#define WHEEL_HALL_PIN 4 // wheel speed hall sensor
#define ENCODER_A 27
#define ENCODER_B 14
#define BRAKE_SENSOR_PIN 39

#define LIMIT_SWITCH_PIN 25
#define MANUAL_MODE_PIN 34

#define LOG_RX 16 // UART RX for Logging
#define LOG_TX 17 // UART TX for Logging


#define clamp(x, min, max) (x < min ? min : x > max ? max : x)
#define lerp(a, b, k) (a + (b - a) * k)