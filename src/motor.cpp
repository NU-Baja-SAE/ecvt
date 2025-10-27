#include "motor.h"
#include <Arduino.h>
#include "pins.h"
#include "potentiometer.h"
// functions: motor setup and motor direction

#define LEDC_CHANNEL 0

void setup_motor()
{
  // setup direction pin
  pinMode(DIRECTION_PIN, OUTPUT);

  // Seting up PWM
  ledcSetup(LEDC_CHANNEL, 1500, 8);
  ledcAttachPin(PWM_PIN, LEDC_CHANNEL);
}


// sets the direction and speed of the motor
// Positive motor_speed values move the sheave together
void set_direction_speed(int motor_speed)
{

  // int pos = read_pos();
  // if (pos > POS_MAX) // if the sheave is at the lowest position, don't let it go lower
  // {
  //   if (motor_speed > 0)
  //   {
  //     motor_speed = 0;
  //   }
  // }
  // if (pos < POS_MIN) // if the sheave is at the highest position, don't let it go higher
  // {
  //   if (motor_speed < 0)
  //   {
  //     motor_speed = 0;
  //   }
  // }


  if (motor_speed < 0)
  {
    digitalWrite(DIRECTION_PIN, LOW);
    motor_speed = -motor_speed;
  }
  else
  {
    digitalWrite(DIRECTION_PIN, HIGH);
  }

  if (motor_speed > 255)
  {
    motor_speed = 255;
  }


  ledcWrite(LEDC_CHANNEL, motor_speed);
}