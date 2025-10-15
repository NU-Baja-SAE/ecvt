#include "pedal_sensors.h"

static void* brake_timer_id; //TODO: is this right??

PedalSensor::PedalSensor(uint8_t pin) 
    : _pin{pin}
{
}

void PedalSensor::read_value()
{
     float val = clamp(1.0 - (analogRead(_pin) - BRAKE_LOW) / (BRAKE_HIGH - BRAKE_LOW), 0.0, 1.0);
     _buffer[_buffer_idx] = val;
    _buffer_idx = _buffer_idx == 0 ? BUFFER_SIZE : _buffer_idx - 1;
#ifdef PEDAL_DEBUG_ACTIVE
    Serial.printf(">brake: %f\n", val);
#endif
}

void setup_pedals()
{
    brake_pedal = PedalSensor{BRAKE_SENSOR_PIN};
    // gas_pedal = PedalSensor{GAS_SENSOR_PIN};

    pinMode(BRAKE_SENSOR_PIN, INPUT);
    // pinMode(GAS_SENSOR_PIN, INPUT);

    xTimerCreate("brake_timer",
                 pdMS_TO_TICKS(BRAKE_TIMER_RATE),
                 pdTRUE,
                 brake_timer_id,
                 brakeTimerCallback
    );      
}

// INVARIANT: idx must be less than BUFFER_SIZE
float PedalSensor::get_value(uint8_t idx=0) {
    return _buffer[(_buffer_idx + idx) % BUFFER_SIZE];
}

// INVARIANT: ticks must be less than BUFFER_SIZE
float PedalSensor::get_change(uint8_t ticks) {
    return get_value(ticks) - get_value(0);
}

void brakeTimerCallback(TimerHandle_t xTimer) {
    brake_pedal.read_value();
}