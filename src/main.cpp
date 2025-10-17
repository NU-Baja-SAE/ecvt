#include <Arduino.h>
#include "motor.h"
#include "potentiometer.h"
#include "pins.h"
#include "pid.h"
#include "pulseCounter.h"
#include "pedal_sensors.h"
#include "wheelSpeed.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  //configure whether pins will be sending or receiving data
  setup_motor();
  Serial.println("setup motor finished");
  
  setup_potentiometer();
  Serial.println("setup potentiometer finished");

  setup_pedals();
  Serial.println("setup pedal sensors finish");

  setup_pid_task();
  Serial.println("setup pid finished");  
  
  init_pulse_counter();
  Serial.println("setup hall pulse counter finished");

  init_wheel_speed();
  Serial.println("setup wheel speed counter finished");

  Serial.println("Setup Complete");

}

void loop() {
  delay(100);
}

// put function definitions here:

