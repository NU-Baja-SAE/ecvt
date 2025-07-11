#include <cstdint>
#include <Arduino.h>
#include <vector>

#define PEDAL_DEBUG_ACTIVE // comment this out to disable debug printing

#define BRAKE_SENSOR_PIN 0 //TODO
#define GAS_SENSOR_PIN 0 //TODO

void setup_pedals();
void read_pedals_task(void* pvParameters);

enum class PEDAL_STATE // just a bool for now, but lets us get fancy later
{
    NORMAL = 0,
    SLAMMED
};

class PedalSensor 
{
public:
    PedalSensor(uint8_t pin);

    const char* get_name() {return _name;};
    float get_value() {return _value;}; // returns value between 0.0 and 1.0 for no pressure->full pressure

    void read_value(); // updates the value internally; not a getter

protected:
    const char* _name;
    float _value;
    uint8_t _pin;

};

//TODO: these globals are kind of ugly--alternate implementation?
static PedalSensor brake_pedal{BRAKE_SENSOR_PIN};
static PedalSensor gas_pedal{GAS_SENSOR_PIN};