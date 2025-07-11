#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>
#include "pulseCounter.h"
#include "pedal_sensors.h"

// SECTION: Engine RPM Constants
// #define IDLE_RPM 500
// #define MAX_RPM 1200
// #define SETPOINT_RPM 800
#define IDLE_RPM 1900
#define MAX_RPM 3800
#define OPTIMAL_RPM 3000

#define MAX_SHEAVE_SETPOINT 220 
#define IDLE_SHEAVE_SETPOINT -90
#define LOW_SHEAVE_SETPOINT -65
#define LOW_MAX_SETPOINT 50

#define BRAKE_CHANGE_THRESHOLD 0.1 //TODO: how much of a change in one cycle should be treated as slamming the brakes
#define BRAKE_RESET_THRESHOLD 0.5 //TODO: value above which the brakes should not be considered slammed anymore, even if we haven't reached low gear

// SECTION: Global Variables
int _vel_setpoint = 0;
ESP32Encoder encoder;

// SECTION: Function Prototypes
void pid_loop_task(void *pvParameters);
float calculate_setpoint(float rpm, float sheave_setpoint);
float moving_average(float newVal, float *arr, int n, int *index);

// sets up a freertos task for the pid loop
void setup_pid_task()
{
    encoder.attachHalfQuad(ENCODER_A, ENCODER_B);

    int analogValue = analogRead(POT_PIN);
    int pos = map(analogValue, 0, 4095, -230, 230);
    // 3835 = 146
    // 1361 = -176

    encoder.setCount(pos);

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

    float last_brake_percent = brake_pedal.get_value();
    float brake_percent = brake_pedal.get_value();

    PEDAL_STATE brakes_state = PEDAL_STATE::NORMAL;

    while (1)
    {
        float rpm = get_engine_rpm();
        rpm = moving_average(rpm, filter_array_rpm, FILTER_SIZE, &filter_index_rpm);

        brake_percent = brake_pedal.get_value();
        
        if (brake_percent - last_brake_percent > BRAKE_CHANGE_THRESHOLD) // if we've slammed on the brakes, switch to that mode
        {
            brakes_state = PEDAL_STATE::SLAMMED;
        }

        if (brakes_state == PEDAL_STATE::SLAMMED) // slammed brakes means straight to low gear
        {
            setpoint = LOW_SHEAVE_SETPOINT;
        } 
        else
        {
            // int targetRPM = map(analogRead(36), 0, 4095, 500, 1200);
            setpoint = calculate_setpoint(rpm, setpoint);
            // setpoint = map(analogRead(36), 0, 4095, IDLE_SHEAVE_SETPOINT, MAX_SHEAVE_SETPOINT);
            // Serial.printf(">manualSetpoint: %d\n", map(analogRead(36), 0, 4095, IDLE_SHEAVE_SETPOINT, MAX_SHEAVE_SETPOINT));
        }

        int pos = encoder.getCount();

        if(brakes_state == PEDAL_STATE::SLAMMED && (pos < LOW_MAX_SETPOINT || brake_percent > BRAKE_RESET_THRESHOLD)) // if we've reached low gear or backed off the brake, switch back to regular mode
        {
            brakes_state = PEDAL_STATE::NORMAL;
        }

        last_brake_percent = brake_percent; // update brake percent

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
        // Serial.printf(">analog: %d\n", analogRead(POT_PIN));
        // Serial.printf(">derivative: %f\n", derivative * POS_Kd);
        // Serial.printf(">integral: %f\n", integral * POS_Ki);
        Serial.printf(">rpm: %f\n", rpm);
        // Serial.printf(">targetRPM: %d\n", targetRPM);

        // Serial.printf(">count: %d\n", get_pulse_counter());
        // Serial.printf(">deltaCount: %f\n", deltaCount);
        // Serial.printf(">deltaT: %f\n", deltaT);
        delay(1);
    }
}

float calculate_setpoint(float rpm, float sheave_setpoint)
{
    static float last_Error = 0;
    
    if (rpm < IDLE_RPM) // if the rpm is less than the idle rpm
    {
        return IDLE_SHEAVE_SETPOINT;
    }
    // else if (rpm > MAX_RPM) // if the rpm is greater than the max rpm
    // {
    //     return MAX_SHEAVE_SETPOINT;
    // }
    else // P controller for RPM setpoint
    {
        float rpmError = OPTIMAL_RPM - rpm; // positive error means the rpm is too low
        
        float d_error = last_Error - rpmError; // Derivative error


        float d_setpoint = -rpmError * RPM_Kp + d_error * RPM_Kd; // negative because lower rpm means more negative sheve position position

        float low_setpoint = lerp(LOW_SHEAVE_SETPOINT, LOW_MAX_SETPOINT, (rpm - IDLE_RPM) / (MAX_RPM - IDLE_RPM));

        low_setpoint = clamp(low_setpoint, LOW_SHEAVE_SETPOINT, LOW_MAX_SETPOINT);

        last_Error = rpmError;

        return clamp(sheave_setpoint + d_setpoint, low_setpoint, MAX_SHEAVE_SETPOINT);
    }
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