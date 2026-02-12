#include "wheelSpeed.h"
#include <driver/pcnt.h>
#include "pins.h"
#include <Arduino.h>

pcnt_config_t wheel_pcnt_config;

// pins and unit for wheel hall sensor
pcnt_unit_t wheel_counter_id = PCNT_UNIT_2;
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
    
    gpio_pullup_en((gpio_num_t)WHEEL_HALL_PIN); // enable pullup on hall sensor pin to avoid false triggering

}

int get_wheel_pulse_counter()
{
    int16_t pulse_count;
    pcnt_get_counter_value(wheel_counter_id, &pulse_count);
    return pulse_count;
}

float get_wheel_speed()
{
    const int BUFFER_SIZE = 5;
    static u_int64_t timeBuffer[BUFFER_SIZE] = {0};
    static u_int64_t countBuffer[BUFFER_SIZE] = {0};
    static int bufferIndex = 0;
    static bool bufferFilled = false;
    static float speed = 0;
    static u_int64_t lastUpdateTime = 0;

    // Get current measurements
    u_int64_t currentTime = micros();
    u_int64_t currentCount = get_wheel_pulse_counter();

    // Only update buffer every 100ms
    if ((currentTime - lastUpdateTime) < 100000) // 100ms in microseconds
    {
        return speed;
    }

    lastUpdateTime = currentTime;

    // Store current measurement in circular buffer
    timeBuffer[bufferIndex] = currentTime;
    countBuffer[bufferIndex] = currentCount;

    // Calculate the index of the oldest measurement
    int oldestIndex = (bufferIndex + 1) % BUFFER_SIZE;

    // Mark buffer as filled once we've wrapped around
    if (bufferIndex == BUFFER_SIZE - 1 && !bufferFilled)
    {
        bufferFilled = true;
    }

    // Update buffer index for next call
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    // Only calculate speed once we have enough samples
    if (bufferFilled)
    {
        u_int64_t oldestTime = timeBuffer[oldestIndex];
        u_int64_t oldestCount = countBuffer[oldestIndex];

        float deltaT = (currentTime - oldestTime) / 1000000.0;
        uint64_t deltaCount = currentCount - oldestCount;

        speed = 60.0 * deltaCount / (deltaT * 4.0); // 4 magnets per revolution
    }

    return speed;
}
