#include "potentiometer.h"
#include <Arduino.h>
#include "pins.h"

int position; // Sheave position, reads from 0 to 4095, might scale it to between 0-1000.

void setup_potentiometer()
{
    pinMode(POT_PIN, INPUT);
}


/**
    @brief Reads the potentiometer value and returns it
*/
int read_pos()
{
    int pot_v = analogRead(POT_PIN); // potentiometer lever
    return pot_v;
}
