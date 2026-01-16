#ifndef MANUAL_MODE_H
#define MANUAL_MODE_H

enum Mode_T{
    POWER_MODE, TORQUE_MODE
};
Mode_T mode_read();
// Mode_T car_mode;

#endif // MANUAL_MODE_H
