#include <Arduino.h>


int pidLoopDelay = 1000;


hw_timer_t * pidTimerPos = NULL; 
portMUX_TYPE pidTimerPosMUX = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR pidLoopPos() {
  portENTER_CRITICAL_ISR(&pidTimerPosMUX);
    
    Serial.println("test");
  portEXIT_CRITICAL_ISR(&pidTimerPosMUX);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pidTimerPos = timerBegin(3, 80, true);
  timerAttachInterrupt(pidTimerPos, &pidLoopPos, true);
  timerAlarmWrite(pidTimerPos, pidLoopDelay, true);
  timerAlarmEnable(pidTimerPos);


}



 void loop() {
//     while (pot_v < 3900){


 }

