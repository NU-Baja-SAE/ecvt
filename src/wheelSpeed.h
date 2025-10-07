// Code for wheel speed hall sensor for Clank (only works for 1 wheel)
// Assumes latching hall sensor and alternating polarity magnet so 1 edge per magnet.

void init_wheel_speed();
int get_wheel_pulse_counter();
float get_wheel_speed();
