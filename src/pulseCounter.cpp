#include "pulseCounter.h"
#include <esp32_pcnt.h>
#include "pins.h"

// create counter object
PulseCounter pc0;

void init_pulse_counter()
{
    pcnt_isr_service_install(0);
    // setup hardware pulse counters
    // initialise counter unit 0, channel 0 with signal input GPIO pin and control signal input pin (0 = no control signal input)
    pc0.initialise(HALL_OUTPUT_PIN, PCNT_PIN_NOT_USED);

    // count up on both edges
    pc0.set_mode(PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);

    // set glitch filter to ignore pulses less than 1000 x 2.5ns
    pc0.set_filter_value(1000);

    // clear and restart the counter
    pc0.clear();
    pc0.resume();
}

int get_pulse_counter()
{
    return pc0.get_value();
}

float get_engine_rpm()
{
    static u_int64_t lastTime = micros();
    static u_int64_t lastCount = get_pulse_counter();
    static float rpm = 0;
    
    // Engine RPM calculation
    u_int64_t currentTime = micros();
    u_int64_t currentCount = get_pulse_counter();
    float deltaT = (currentTime - lastTime) / 1000000.0;
    if (deltaT > 0.05)
    {
        uint64_t deltaCount = currentCount - lastCount;
        Serial.printf(">deltaCount: %d\n", deltaCount);
        rpm = 60.0 * deltaCount / (deltaT * 6.0); // 6 pulses per revolution

        lastTime = currentTime;
        lastCount = currentCount;
    }
    return rpm;
}
