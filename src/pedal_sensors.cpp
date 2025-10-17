#include "pedal_sensors.h"

static void* brake_timer_id = (void*) 0; //TODO: is this right??

PedalSensor brake_pedal(BRAKE_SENSOR_PIN);

PedalSensor::PedalSensor(uint8_t pin) 
    : _pin{pin}
{
}

void PedalSensor::read_value()
{
    if (_buffer_idx == 0) {
        _buffer_idx = BUFFER_SIZE;
    }
    _buffer_idx -= 1;
    float val = clamp(1.0 - (float)(analogRead(_pin) - BRAKE_LOW) / (float)(BRAKE_HIGH - BRAKE_LOW), 0.0, 1.0);
    _buffer[_buffer_idx] = val;
#ifdef PEDAL_DEBUG_ACTIVE
    Serial.printf(">brake: %f\n", val);
#endif
}

void brakeTimerCallback(TimerHandle_t xTimer) {
    brake_pedal.read_value();
}

void setup_pedals()
{
    pinMode(BRAKE_SENSOR_PIN, INPUT);
    // pinMode(GAS_SENSOR_PIN, INPUT);

    TimerHandle_t brake_timer = xTimerCreate("brake_timer",
                                             pdMS_TO_TICKS(BRAKE_TIMER_RATE),
                                             pdTRUE,
                                             brake_timer_id,
                                             brakeTimerCallback
    );

    if (!brake_timer) {
        Serial.printf("ERROR: Brake timer could not be created\n");
    }

    if (xTimerStart(brake_timer, 0) != pdPASS) {
        Serial.printf("ERROR: Brake timer could not be started\n");
    }
}

// INVARIANT: idx must be less than BUFFER_SIZE
float PedalSensor::get_value(uint8_t idx) {
    return _buffer[(_buffer_idx + idx) % BUFFER_SIZE];
}

// INVARIANT: ticks must be less than BUFFER_SIZE
float PedalSensor::get_change(uint8_t ticks) {
    return get_value(0) - get_value(ticks);
}