#include <Arduino.h>
#include "motor.h"
#include "pins.h"
#include "potentiometer.h"
#include "pid.h"
#include <ESP32Encoder.h>
#include "pulseCounter.h"
#include "pedal_sensors.h"
#include "wheelSpeed.h"

#define LOG_HEADER "time_us,engine_rpm,wheel_rpm,secondary_rpm,sheave_position,sheave_setpoint,pwm,brake_position,kp_term,ki_term,kd_term"
#include "manual_mode.h"

// SECTION: Debug Defines
// Uncomment for full debug
#define TELEPLOT_PID_DEBUG 1

// SECTION: Engine RPM Constants
// #define IDLE_RPM 500
// #define MAX_RPM 1200
// #define SETPOINT_RPM 800
#define IDLE_RPM 1900
#define MAX_RPM 3800

#define MAX_SHEAVE_SETPOINT 300
#define IDLE_SHEAVE_SETPOINT -90
#define LOW_SHEAVE_SETPOINT -35
#define LOW_MAX_SETPOINT 50

// TODO: tune these
#define BRAKE_SLAM_TICKS 100      // ms, the change must happen in this many ticks
#define BRAKE_SLAM_THRESHOLD 0.5  // below this threshold, we never slam
#define BRAKE_SLAM_CHANGE 0.2     // change that must occur within time
#define BRAKE_RESET_THRESHOLD 0.4 // value below which the brakes should not be considered slammed anymore, even if we haven't reached low gear
/* since there are a lot of things here, just to clarify:
if the brakes change by SLAM_CHANGE within SLAM_TICKS ticks, and the new value
is below SLAM_THRESHOLD, we stay in slam mode until we reach low gear or the brakes go
above RESET_THRESHOLD*/

// SECTION: Global Variables
int _vel_setpoint = 0;
ESP32Encoder encoder;

// setup serial uart
HardwareSerial SerialUART1(1); // Use UART1 for UART data logging

// SECTION: Function Prototypes
void pid_loop_task(void *pvParameters);
float calculate_setpoint(float rpm, float sheave_setpoint);
float moving_average(float newVal, float *arr, int n, int *index);
void home_motor();

// sets up a freertos task for the pid loop
void setup_pid_task()
{
    encoder.attachHalfQuad(ENCODER_A, ENCODER_B);

    home_motor(); // home motor to set zero position
    encoder.setCount(IDLE_SHEAVE_SETPOINT);

    // Setup Serial UART for logging
    SerialUART1.begin(115200, SERIAL_8N1, LOG_RX, LOG_TX);
    SerialUART1.println(LOG_HEADER);

    xTaskCreate(pid_loop_task,   // Function to implement the task
                "pid_loop_task", // A name just for humans
                10000,           // This stack size can be checked & adjusted by reading the Stack Highwater
                NULL,            // Parameters to pass to the task
                1,               // This priority can be adjusted
                NULL);           // Task handle. Not used here
}

// for exponential smoothing, not currently used
#define ALPHA 0.8
#define D_ALPHA 0.8

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
    float error = 0;
    float derivative = 0;

    float setpoint = IDLE_SHEAVE_SETPOINT;
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

    PEDAL_STATE brakes_state = PEDAL_STATE::NORMAL;

    int negative_error_time = 0; // time spent with negative error

    while (1)
    {
        float rpm = get_engine_rpm();
        float secondary_rpm = get_secondary_rpm();
        rpm = moving_average(rpm, filter_array_rpm, FILTER_SIZE, &filter_index_rpm);

        // TODO: Should brake slam or manual mode take precedence?
        if ((brakes_state != PEDAL_STATE::SLAMMED) &&                             // if we're not already slammed
            (brake_pedal.get_change(BRAKE_SLAM_TICKS - 1) > BRAKE_SLAM_CHANGE) && // and the change has happened fast
            (brake_pedal.get_value(0) > BRAKE_SLAM_THRESHOLD))                    // and we're past the threshold
        {
            brakes_state = PEDAL_STATE::SLAMMED; //... slam on the brakes

#ifdef PEDAL_DEBUG_ACTIVE
            Serial.printf("BRAKES SLAMMED!\n");
#endif
        }

        if (brakes_state == PEDAL_STATE::SLAMMED) // slammed brakes means straight to low gear
        {
            setpoint = LOW_SHEAVE_SETPOINT;
        }
        else
        {
            setpoint = calculate_setpoint(rpm, setpoint);
        }
        float wheel_speed = get_wheel_speed();

        int pos = encoder.getCount();

        // TODO: What should the condition for checking the position be?
        if (brakes_state == PEDAL_STATE::SLAMMED && (pos < LOW_MAX_SETPOINT || brake_pedal.get_value(0) < BRAKE_RESET_THRESHOLD)) // if we've reached low gear or backed off the brake, switch back to regular mode
        {
            brakes_state = PEDAL_STATE::NORMAL;
            Serial.printf("Brakes deslammed\n");
        }

        if (brakes_state == PEDAL_STATE::NORMAL)
        {
            error = setpoint - (float)pos; // calculate the error

            derivative = error - last_error; // Derivatice calculation
            last_error = error;

            derivative = moving_average(derivative, filter_array_derivative, 5, &filter_index_derivative);

            integral += error; // I controller calculation
            integral = clamp(integral, 0, POS_MAX_I_TERM);

            if (error >= 0)
            {
                result = error * POS_Kp + integral * POS_Ki + derivative * POS_Kd; // PI controller calculation
            }
            else
            {                                                                                      // use different gains for negative error
                result = error * POS_Kp * Kp_multiplier + integral * POS_Ki + derivative * POS_Kd; // PI controller calculation
            }

            if (error < 0)
            {
                negative_error_time++;
            }
            else
            {
                negative_error_time = 0;
            }

            // to prevent changing motor direction unnecessarily
            if ((result <= 0.0) && (result > -15.0) && (negative_error_time < 200))
            {
                result = 1;
            }
            if ((result <= 0.0) && (result > -7.5))
            { // never go below 7.5 in the negative direction
                result = 1;
            }

            set_direction_speed((int)result); // set the motor speed based on the pid term
        }
        else if (brakes_state == PEDAL_STATE::SLAMMED)
        {
            set_direction_speed(-255);
        }

// print these so teleplot can read them
#ifdef TELEPLOT_PID_DEBUG
        Serial.printf(">rpm: %d\n", (int)rpm);
        Serial.printf(">pos: %d\n", (int)pos);
        Serial.printf(">wheel_speed: %f\n", wheel_speed);
        Serial.printf(">secondary_rpm: %f\n", secondary_rpm);
        Serial.printf(">pos_setpoint: %f\n", setpoint);
        Serial.printf(">result: %f\n", clamp(result, -255, 255));
#endif

        static int counter = 0;
        counter++;
        if (counter >= 20) // every 20 loops
        {
            counter = 0;
            SerialUART1.printf("%lld, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                               esp_timer_get_time(),
                               rpm,
                               wheel_speed,
                               secondary_rpm,
                               (float)pos,
                               setpoint,
                               result,
                               brake_pedal.get_value(0),
                               error * POS_Kp,
                               integral * POS_Ki,
                               derivative * POS_Kd);
        }
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
        
        Mode_T car_mode = mode_read();
        int optimal_rpm = 3000;
        
        if (car_mode == TORQUE_MODE) {
            optimal_rpm = 2400;
        }
        else if (car_mode == POWER_MODE) {
            optimal_rpm = 3000;
        }
        float rpmError = optimal_rpm - rpm; // positive error means the rpm is too low

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

#define MAX_TIME 10000 // max time to home in ms
void home_motor()
{
    Serial.println("Homing motor...");
    // move to the limit switch
    int pos = encoder.getCount();
    int setpoint = pos;
    int integral = 0;

    int time = millis();

    // move backwards until we hit the limit switch
    while (digitalRead(LIMIT_SWITCH_PIN) == HIGH)
    {
        setpoint -= 1;
        int error = setpoint - encoder.getCount();
        integral += error;
        integral = clamp(integral, 0, POS_MAX_I_TERM);
        int result = error * POS_Kp + integral * POS_Ki;
        result = clamp(result, -50, 50); // limit speed
        set_direction_speed(result);
        Serial.printf("Homing... pos: %d, setpoint: %d, error: %d, result: %d\n", encoder.getCount(), setpoint, error, result);
        delay(50);

        if (millis() - time > MAX_TIME)
        {
            Serial.println("Homing timed out!");
            set_direction_speed(0);
            while (1)
            {
            }
        }
    }
    set_direction_speed(0);
    // move forward a bit to get off the limit switch
    delay(1000);
    setpoint = encoder.getCount() + 25;
    integral = 0;
    // move forward under PID control to an offset position away from the limit switch
    while (setpoint > encoder.getCount() - 5) // add a small deadzone
    {
        int error = setpoint - encoder.getCount();
        integral += error;
        integral = clamp(integral, 0, POS_MAX_I_TERM);

        int result = error * POS_Kp + integral * POS_Ki;
        result = clamp(result, -50, 50); // limit speed
        set_direction_speed(result);
        Serial.printf("Homing... pos: %d, setpoint: %d, error: %d, result: %d\n", encoder.getCount(), setpoint, error, result);

        delay(50);
        if (millis() - time > MAX_TIME)
        {
            Serial.println("Homing timed out!");
            set_direction_speed(0);
            while (1)
            {
            }
        }
    }
    set_direction_speed(0);
    delay(1000);

    // move back to the limit switch slowly
    while (digitalRead(LIMIT_SWITCH_PIN) == HIGH)
    {
        setpoint -= 1;
        int error = setpoint - encoder.getCount();
        integral += error;
        integral = clamp(integral, 0, POS_MAX_I_TERM);

        int result = error * POS_Kp + integral * POS_Ki;
        result = clamp(result, -50, 50); // limit speed for final approach
        set_direction_speed(result);
        Serial.printf("Homing... pos: %d, setpoint: %d, error: %d, result: %d\n", encoder.getCount(), setpoint, error, result);

        delay(50);
        if (millis() - time > MAX_TIME)
        {
            Serial.println("Homing timed out!");
            set_direction_speed(0);
            while (1)
            {
            }
        }
    }

    set_direction_speed(0);
    delay(1000);
}