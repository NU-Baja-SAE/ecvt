#include <cstdint>
#include <Arduino.h>
#include <vector>
#include "pins.h"

#define PEDAL_DEBUG_ACTIVE 1 // comment this out to disable debug printing

#define BRAKE_LOW 2600 // maximally depressed value (padded; empirically around 2800)
#define BRAKE_HIGH 4095 // unpressed value

#define BRAKE_TIMER_RATE 100 // ms

#define BUFFER_SIZE 10

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
    float get_value(uint8_t idx=0); // returns value N ticks ago between 0.0 and 1.0 for no pressure->full pressure
    float get_change(uint8_t ticks); // return change since the N ticks ago

    void read_value(); // updates the value internally; NOT a getter

protected:
    const char* _name;
    float _value; // this is the RAW potentiometer value
    uint8_t _pin;
    float _buffer[BUFFER_SIZE] = {0.0};
    uint8_t _buffer_idx = 0;
};

//TODO: these globals are kind of ugly--alternate implementation?
extern PedalSensor brake_pedal;
// static PedalSensor gas_pedal{GAS_SENSOR_PIN};