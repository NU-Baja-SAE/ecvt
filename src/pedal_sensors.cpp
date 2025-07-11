#include "pedal_sensors.h"

PedalSensor::PedalSensor(uint8_t pin) 
    : _pin{pin}
{
}

void PedalSensor::read_value()
{
    _value = analogRead(_pin); //TODO: Presumably we should scale this
}

void setup_pedals()
{
    brake_pedal = PedalSensor{BRAKE_SENSOR_PIN};
    gas_pedal = PedalSensor{GAS_SENSOR_PIN};

    pinMode(BRAKE_SENSOR_PIN, INPUT);
    pinMode(GAS_SENSOR_PIN, INPUT);

    xTaskCreate(read_pedals_task,   // Function to implement the task
            "read_pedals_task", // A name just for humans
            1000,            // This stack size can be checked & adjusted by reading the Stack Highwater
            NULL,            // Parameters to pass to the task
            1,               // This priority can be adjusted
            NULL);           // Task handle. Not used here
}

void read_pedals_task(void *pvParameters)
{
    while(1)
    {
        brake_pedal.read_value();
        gas_pedal.read_value();

#ifdef PEDAL_DEBUG_ACTIVE
        Serial.printf(">brake: %.3f\n", brake_pedal.get_value());
        Serial.printf(">gas: %.3f\n", gas_pedal.get_value());
#endif

        delay(1);
    }
}