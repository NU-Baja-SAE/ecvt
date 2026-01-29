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
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // Seting up PWM
  ledcSetup(LEDC_CHANNEL, 1500, 8);
  ledcAttachPin(PWM_PIN, LEDC_CHANNEL);
}


// sets the direction and speed of the motor
// Positive motor_speed values move the sheave together
void set_direction_speed(int motor_speed)
{

  // check if limit switch is pressed when trying to move further back
  if ((digitalRead(LIMIT_SWITCH_PIN) == LOW) && (motor_speed < 0))
  {
    motor_speed = 0; // stop the motor
    Serial.println("edge case entered\n");
  }

  if (motor_speed < 0)
  {
    digitalWrite(DIRECTION_PIN, LOW);
    motor_speed = -motor_speed;
    Serial.println("speed: %d, dir: neg\n", motor_speed);
  }
  else
  {
    digitalWrite(DIRECTION_PIN, HIGH);
    Serial.println("speed: %d, dir: pos\n", motor_speed);
  }

  if (motor_speed > 255)
  {
    motor_speed = 255;
    Serial.println("speed clamped\n");
  }


  ledcWrite(LEDC_CHANNEL, motor_speed);
  Serial.println("motor spinning at speed %d\n", motor_speed);
}