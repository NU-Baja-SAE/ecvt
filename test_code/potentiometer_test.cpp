#include <Arduino.h>
#include "motor.h"

// warning i do not know c/c++ however i do know arduino (ish) so that what i wrote this in but its taking issue with a lot of the things im 
//doing and i don't know if its a me issue or an ide-wants-c/c++ issue. nevertheless it is something. 


uint16_t pot_v; // measurement across sliding part of potentiometer
uint16_t total_v; //measuremetn across potentiometer used as resistor
uint8_t position; //zero value based on sheave travel (may not be full retraction of potentiometer)
uint8_t positionThousand; //1000 value based on sheave travel (may not be full extension of potentiometer)

uint8_t potPinOne = 32; // measures at potentiometer slider

 void readPos() {
    pot_v = analogRead(potPinOne); // potentiometer lever
// total_v = analogRead(potPinTwo); // whole resistor

// read the voltage across the potentiometer as a proportion, because when the voltage changes as the battery drains, the proportion will remain constant 
   // encoderReading = pot_v / total_v; // find how far along the resistor the potentiometer is as a fraction
    position = pot_v; // scale to make more workable number based on zero/thousand setpoints from calibration
    Serial.println("position read");
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(potPinOne, INPUT);


  setup_motor();
  Serial.println("motor setup complete");

//   pidTimerPos = timerBegin(3, 80, true);
//   timerAttachInterrupt(pidTimerPos, &pidLoopPos, true);
//   timerAlarmWrite(pidTimerPos, pidLoopDelay, true);
//   timerAlarmEnable(pidTimerPos);
  Serial.println("timer setup complete");

  
}






// tested direction switchin code in cole's test setup:

 void loop() {
//     while (pot_v < 3900){
//         Serial.println(pot_v);
//         Serial.println("Increasing");
//         digitalWrite(directionPin, HIGH);
//         analogWrite(pwmPin, 50);
//         pot_v = analogRead(potPinOne);
//     }

//     analogWrite(pwmPin, 0);
//     delay(1000);

//     while (pot_v > 300){
//         Serial.println(pot_v);
//         Serial.println("Decreasing");
//         digitalWrite(directionPin, LOW);
//         analogWrite(pwmPin, 50);
//         pot_v = analogRead(potPinOne);
//     }

//     analogWrite(pwmPin, 0);
    delay(100);
    readPos();
    Serial.printf(">:position: %d \n", position);
 }

