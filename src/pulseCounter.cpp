#include "pulseCounter.h"
#include <driver/pcnt.h>
#include "pins.h"
#include <Arduino.h>

pcnt_config_t primary_pcnt_config;

// pins and unit for primary hall sensor
pcnt_unit_t primary_counter_id = PCNT_UNIT_1;

pcnt_config_t secondary_pcnt_config;

// pins and unit for primary hall sensor
pcnt_unit_t secondary_counter_id = PCNT_UNIT_3;

// initialize the primary pulse counter
void init_pulse_counter()
{
    // configure counter for primary
    primary_pcnt_config.unit = primary_counter_id;
    primary_pcnt_config.channel = PCNT_CHANNEL_0; // only use channel 0 of each counter unit.
    // Set signal and control inputGPIOs
    primary_pcnt_config.pulse_gpio_num = PRIMARY_HALL_PIN;
    primary_pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    pcnt_unit_config(&primary_pcnt_config);

    // set counting mode for primary counter, count both rising and falling edges
    pcnt_set_mode(primary_counter_id, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);

    // set filter value to ignore glitches, this is in units of APB clock cycles, so for 80MHz clock, 1000 = 12.5us
    pcnt_set_filter_value(primary_counter_id, 1000);
    pcnt_filter_enable(primary_counter_id);

    // clear and start the counter
    pcnt_counter_clear(primary_counter_id);
    pcnt_counter_resume(primary_counter_id);

    // configure counter for secondary
    secondary_pcnt_config.unit = secondary_counter_id;
    secondary_pcnt_config.channel = PCNT_CHANNEL_0; // only use channel 0 of each counter unit.
    // Set signal and control inputGPIOs
    secondary_pcnt_config.pulse_gpio_num = SECONDARY_HALL_PIN;
    secondary_pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    pcnt_unit_config(&secondary_pcnt_config);

    // set counting mode for secondary counter, count both rising and falling edges
    esp_err_t err = pcnt_set_mode(secondary_counter_id, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);
    if (err != ESP_OK)
    {
        Serial.printf("Error setting mode for secondary counter: %d\n", err);
    }

    // set filter value to ignore glitches, this is in units of APB clock cycles, so for 80MHz clock, 1000 = 12.5us
    pcnt_set_filter_value(secondary_counter_id, 1000);
    pcnt_filter_enable(secondary_counter_id);

    // clear and start the counter
    pcnt_counter_clear(secondary_counter_id);
    pcnt_counter_resume(secondary_counter_id);

    gpio_pullup_en((gpio_num_t)SECONDARY_HALL_PIN); // enable pulldown on hall sensor pin to avoid false triggering
}

// get the current count of the primary pulse counter
int get_primary_counter()
{
    int16_t pulse_count;
    pcnt_get_counter_value(primary_counter_id, &pulse_count);
    return pulse_count;
}

// get the current count of the primary pulse counter
int get_secondary_counter()
{
    int16_t pulse_count = -1;
    pcnt_get_counter_value(secondary_counter_id, &pulse_count);
    return pulse_count;
}

// calculate engine RPM from primary pulse counter
float get_engine_rpm()
{
    static u_int64_t lastTime = micros();
    static u_int64_t lastCount = get_primary_counter();
    static float rpm = 0;

    // Engine RPM calculation
    u_int64_t currentTime = micros();
    u_int64_t currentCount = get_primary_counter();
    float deltaT = (currentTime - lastTime) / 1000000.0;
    if (deltaT > 0.05)
    {
        uint64_t deltaCount = currentCount - lastCount;
        rpm = 60.0 * deltaCount / (deltaT * 6.0); // 6 pulses per revolution

        lastTime = currentTime;
        lastCount = currentCount;
    }
    return rpm;
}

// calculate engine RPM from secondary pulse counter
float get_secondary_rpm()
{
    // return get_secondary_counter();
    static u_int64_t lastTime = micros();
    static u_int64_t lastCount = get_secondary_counter();
    static float rpm = 0;

    // Engine RPM calculation
    u_int64_t currentTime = micros();
    u_int64_t currentCount = get_secondary_counter();
    float deltaT = (currentTime - lastTime) / 1000000.0;
    if (deltaT > 0.05)
    {
        uint64_t deltaCount = currentCount - lastCount;
        rpm = 60.0 * deltaCount / (deltaT * 6.0); // 6 pulses per revolution

        lastTime = currentTime;
        lastCount = currentCount;
    }
    return rpm;
}