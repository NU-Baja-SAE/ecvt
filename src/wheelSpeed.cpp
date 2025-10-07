#include "wheelSpeed.h"
#include <esp32_pcnt.h>
#include "pins.h"

// create counter object
PulseCounter pc1;

void init_wheel_speed()
{
    // setup hardware pulse counter
    // initialise counter unit 1, channel 0 with signal input GPIO pin and control signal input pin (0 = no control signal input)
    pc1.initialise(WHEEL_HALL_PIN, PCNT_PIN_NOT_USED);

    // count up on both edges
    pc1.set_mode(PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);

    // set glitch filter to ignore pulses less than 1000 x 2.5ns
    pc1.set_filter_value(1000);

    // clear and restart the counter
    pc1.clear();
    pc1.resume();
}

int get_wheel_pulse_counter()
{
    return pc1.get_value();
}

float get_wheel_speed()
{
    static u_int64_t lastTime = micros();
    static u_int64_t lastCount = get_wheel_pulse_counter();
    static float speed = 0;

    // Wheel speed calculation
    u_int64_t currentTime = micros();
    u_int64_t currentCount = get_wheel_pulse_counter();
    float deltaT = (currentTime - lastTime) / 1000000.0;
    if (deltaT > 0.05)
    {
        uint64_t deltaCount = currentCount - lastCount;
        Serial.printf(">wheelDeltaCount: %d\n", deltaCount);
        speed = 60.0 * deltaCount / (deltaT * 4.0); // 4 magnets per revolution

        lastTime = currentTime;
        lastCount = currentCount;
    }
    return speed;
}
