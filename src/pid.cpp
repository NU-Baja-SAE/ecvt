#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>
#include "pulseCounter.h"

// SECTION: Engine RPM Constants
// #define IDLE_RPM 500
// #define MAX_RPM 1200
// #define SETPOINT_RPM 800
#define IDLE_RPM 1900
#define MAX_RPM 3800
#define TARGET_RPM 3000


#define MAX_SHEAVE_SETPOINT 220 
#define IDLE_SHEAVE_SETPOINT -90
#define LOW_SHEAVE_SETPOINT -65
#define LOW_MAX_SETPOINT 50

// SECTION: Global Variables
int _vel_setpoint = 0;
ESP32Encoder encoder;

// setup serial uart on pin 16 and 17
HardwareSerial SerialUART1(1); // Use UART1 for UART data logging

// SECTION: Function Prototypes
void pid_loop_task(void *pvParameters);
float calculate_setpoint(float primary_rpm, float secondary_rpm, float sheave_setpoint);
float moving_average(float newVal, float *arr, int n, int *index);
float gear_ratio_from_secondary_rpm(float x);
float sheave_setpoint_from_gear_ratio(float x);

// sets up a freertos task for the pid loop
void setup_pid_task()
{
    encoder.attachHalfQuad(ENCODER_A, ENCODER_B);

    int analogValue = analogRead(POT_PIN);
    int pos = map(analogValue, 0, 4095, -230, 230);
    // 3835 = 146
    // 1361 = -176

    encoder.setCount(pos);


    SerialUART1.begin(115200, SERIAL_8N1, DEBUG_RX, DEBUG_TX);

    xTaskCreate(pid_loop_task,   // Function to implement the task
                "pid_loop_task", // A name just for humans
                10000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL,            // Parameters to pass to the task
                1,               // This priority can be adjusted
                NULL);           // Task handle. Not used here
}

#define ALPHA 0.8
#define D_ALPHA 0.8

#define clamp(x, min, max) (x < min ? min : x > max ? max \
                                                    : x)
#define lerp(a, b, k) (a + (b - a) * k)

float smoothmin(float a, float b, float k)
{
    float h = clamp(0.5 + 0.5 * (a - b) / k, 0, 1);
    return lerp(a, b, h) - k * h * (1 - h);
}

float smoothmax(float a, float b, float k)
{
    return -smoothmin(-a, -b, k);
}

#define smoothclamp(x, min, max, k) smoothmin(smoothmax(x, min, k), max, k)

void pid_loop_task(void *pvParameters)
{

    float result = 0;
    float integral = 0;

    float setpoint = 0;
    float last_error = 0;

    // moving average filter for the derivative
    float filter_array_derivative[5] = {0, 0, 0, 0, 0};
    int filter_index_derivative = 0;

// moving average filter for the rpm
#define FILTER_SIZE 50
    float filter_array_rpm[FILTER_SIZE];
    for (int i = 0; i < FILTER_SIZE; i++)
        filter_array_rpm[i] = 0;

    int filter_index_rpm = 0;

    while (1)
    {
        float rpm = get_engine_rpm();
        rpm = moving_average(rpm, filter_array_rpm, FILTER_SIZE, &filter_index_rpm);

        setpoint = calculate_setpoint(rpm, 0, setpoint);

        int pos = encoder.getCount();

        float error = setpoint - pos; // calculate the error

        float derivative = error - last_error; // Derivatice calculation
        last_error = error;

        derivative = moving_average(derivative, filter_array_derivative, 5, &filter_index_derivative);

        integral += error; // I controller calculation
        integral = clamp(integral, -POS_MAX_I_TERM, POS_MAX_I_TERM);

        result = error * POS_Kp + integral * POS_Ki + derivative * POS_Kd; // PI controller calculation

        set_direction_speed((int)result); // set the motor speed based on the pid term

        Serial.printf(">pos: %d\n", pos);
        Serial.printf(">pos_setpoint: %f\n", setpoint);
        Serial.printf(">PWM: %f\n", result > 255 ? 255 : result < -255 ? -255
                                                                       : result);
        // Serial.printf(">result: %f\n", result);
        Serial.printf(">rpm: %f\n", rpm);

        static int counter = 0;
        counter++;
        if (counter >= 100) // every 100 loops (about every 100 ms)
        {
            counter = 0;
            SerialUART1.printf("%d, %d\n", (int)rpm, (int)pos);
        }
        delay(1);
    }
}

float calculate_setpoint(float primary_rpm, float secondary_rpm, float sheave_setpoint)
{
    // rpm -> gear ratio
    float gear_ratio = gear_ratio_from_secondary_rpm(secondary_rpm);
    

    // gear ration -> sheave setpoint

}


#define LOW_GEAR 3.6
#define HIGH_GEAR 0.9 
#define SLIP_SPEED 1800 / LOW_GEAR // RPM of the secondary at which the clutch starts to slip
#define CRUISE_LOW 3000 / LOW_GEAR // RPM of the secondary at which we start to shift
#define CRUISE_HIGH 3000 / HIGH_GEAR // RPM of the secondary at which we finish shifting

float gear_ratio_from_secondary_rpm(float x){
    if (x < SLIP_SPEED)
        return LOW_GEAR; // maybe somewhere between LOW and IDLE
    else if (x < CRUISE_LOW)
        return LOW_GEAR;
    else if (x < CRUISE_HIGH)
        return (LOW_GEAR - HIGH_GEAR) / (CRUISE_LOW - CRUISE_HIGH) * (x - CRUISE_LOW) + LOW_GEAR;
    else
        return HIGH_GEAR;
}

float sheave_setpoint_from_gear_ratio(float x){
    if (x > LOW_GEAR)
        return LOW_SHEAVE_SETPOINT; // maybe somewhere between LOW and IDLE
    else if (x > HIGH_GEAR)
        return (LOW_SHEAVE_SETPOINT - MAX_SHEAVE_SETPOINT) / (LOW_GEAR - HIGH_GEAR) * (x - LOW_GEAR) + LOW_SHEAVE_SETPOINT;
    else
        return MAX_SHEAVE_SETPOINT;
}


// moving average filter
// newVal is the new value to add to the array
// arr is a pointer to the array
// n is the number of elements in the array
// index is a pointer to the index of the array
float moving_average(float newVal, float *arr, int n, int *index)
{
    arr[*index] = newVal;
    *index = (*index + 1) % n;

    float sum = 0;
    for (int i = 0; i < n; i++)
    {
        sum += arr[i];
    }

    return sum / n;
}