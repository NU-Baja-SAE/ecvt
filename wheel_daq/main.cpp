#include <Arduino.h>
#include "wheel_rpm.h"

void setup() {
    Serial.begin(115200);
    Serial.println("Wheel DAQ System Starting...");

    setup_wheel_rpm();
    Serial.println("Setup Complete");
}

void loop() {
    float rpm = get_wheel_rpm();

    if (rpm >= 0) {
        Serial.print("Wheel RPM: ");
        Serial.println(rpm);
    }

    delay(100);
}