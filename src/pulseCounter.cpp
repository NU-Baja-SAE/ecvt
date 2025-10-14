#include "pulseCounter.h"
#include <driver/pcnt.h>
#include "pins.h"
#include <Arduino.h>

pcnt_config_t primary_pcnt_config;

// pins and unit for primary hall sensor
pcnt_unit_t primary_counter_id = PCNT_UNIT_0;
int PCNT_INPUT_SIG_IO = PRIMARY_HALL_PIN;
int PCNT_INPUT_CTRL_IO = PCNT_PIN_NOT_USED;


// initialize the primary pulse counter
void init_pulse_counter()
{
    // configure counter for primary
    primary_pcnt_config.unit = primary_counter_id;
    primary_pcnt_config.channel = PCNT_CHANNEL_0; // only use channel 0 of each counter unit.
    // Set signal and control inputGPIOs
    primary_pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
    primary_pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;
    pcnt_unit_config(&primary_pcnt_config);

    // set counting mode for primary counter, count both rising and falling edges
    pcnt_set_mode(primary_counter_id, PCNT_CHANNEL_0, PCNT_COUNT_INC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_KEEP);

    // set filter value to ignore glitches, this is in units of APB clock cycles, so for 80MHz clock, 1000 = 12.5us
    pcnt_set_filter_value(primary_counter_id, 1000); 
    pcnt_filter_enable(primary_counter_id);

    // clear and start the counter
    pcnt_counter_clear(primary_counter_id);
    pcnt_counter_resume(primary_counter_id);
}

// get the current count of the primary pulse counter
int get_primary_counter()
{
    int16_t pulse_count;
    pcnt_get_counter_value(primary_counter_id, &pulse_count);
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
        Serial.printf(">deltaCount: %d\n", deltaCount);
        rpm = 60.0 * deltaCount / (deltaT * 6.0); // 6 pulses per revolution

        lastTime = currentTime;
        lastCount = currentCount;
    }
    return rpm;
}