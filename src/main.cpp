#include <Arduino.h>
#include "motor.h"
#include "hall_sensor.h"
#include "potentiometer.h"
#include "pins.h"
#include "pid.h"
#include "pulseCounter.h"
#include "wheel_rpm.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  //configure whether pins will be sending or receiving data
  setup_motor();
  Serial.println("setup motor finished");
  setup_hall();
  Serial.println("setup hall finished");
  setup_potentiometer();
  Serial.println("setup potentiometer finished");

  setup_pid_task();
  Serial.println("setup pid finished");  
  
  init_pulse_counter();
  Serial.println("setup hall pulse counter finished");

  setup_wheel_rpm();
  Serial.println("setup wheel rpm finished");

  Serial.println("Setup Complete");

}

void loop() {
  static unsigned long last_wheel_update = 0;
  static unsigned long wheel_update_interval = 500; // 500ms intervals

  unsigned long current_time = millis();

  // Only update wheel RPM during longer idle periods
  if (current_time - last_wheel_update >= wheel_update_interval) {
    float wheel_rpm = get_wheel_rpm();
    if (wheel_rpm >= 0) {
      Serial.printf(">wheel_rpm: %.1f\n", wheel_rpm);
    }
    last_wheel_update = current_time;
  }

  delay(100);
}

// put function definitions here:

