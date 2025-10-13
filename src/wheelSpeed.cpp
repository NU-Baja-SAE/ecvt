#include "wheelSpeed.h"
#include <driver/pcnt.h>
#include "pins.h"
#include <Arduino.h>

pcnt_config_t wheel_pcnt_config;

// pins and unit for wheel hall sensor
pcnt_unit_t wheel_counter_id = PCNT_UNIT_1;
int WHEEL_PCNT_INPUT_SIG_IO = WHEEL_HALL_PIN;
int WHEEL_PCNT_INPUT_CTRL_IO = PCNT_PIN_NOT_USED;

void init_wheel_speed()
{
    // configure counter for wheel
    wheel_pcnt_config.unit = wheel_counter_id;
    wheel_pcnt_config.channel = PCNT_CHANNEL_0;
    // Set signal and control input GPIOs
    wheel_pcnt_config.pulse_gpio_num = WHEEL_PCNT_INPUT_SIG_IO;
    wheel_pcnt_config.ctrl_gpio_num = WHEEL_PCNT_INPUT_CTRL_IO;
    pcnt_unit_config(&wheel_pcnt_config);

    // set counting mode for wheel counter, count both rising and falling edges
    pcnt_set_mode(wheel_counter_id, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);

    // set filter value to ignore glitches, this is in units of APB clock cycles, so for 80MHz clock, 1000 = 12.5us
    pcnt_set_filter_value(wheel_counter_id, 1000);
    pcnt_filter_enable(wheel_counter_id);

    // clear and start the counter
    pcnt_counter_clear(wheel_counter_id);
    pcnt_counter_resume(wheel_counter_id);
}

int get_wheel_pulse_counter()
{
    int16_t pulse_count;
    pcnt_get_counter_value(wheel_counter_id, &pulse_count);
    return pulse_count;
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
