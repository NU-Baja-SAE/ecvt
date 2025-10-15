#include <Arduino.h>




// warning i do not know c/c++ however i do know arduino (ish) so that what i wrote this in but its taking issue with a lot of the things im 
//doing and i don't know if its a me issue or an ide-wants-c/c++ issue. nevertheless it is something. 


int hallCount = 0; // Hall sensor counts 


uint8_t hallOutputPin = 32; // reads each time the hall sensor passes a magnet

bool hall_trigger;
int threshold = 250;

// read hall sensor -> figure out threshold value
// count and reset after six counts
// calculate rpm appropriately w/o multisampling



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(hallOutputPin, INPUT);
  hall_trigger = false;
}

void loop() {
  
    int v = analogRead(hallOutputPin);
    if (v > threshold && !hall_trigger) {
      hall_trigger = true;
      hallCount++;

    }
    else if (hall_trigger && v <= threshold) {
      hall_trigger = false;
    }

    // if (v == HIGH) {
    //   hallCount++;
    // }
    Serial.print(">hall_output:");
    Serial.println(v);
    Serial.print(">hall_count:");
    Serial.println(hallCount);
}

 