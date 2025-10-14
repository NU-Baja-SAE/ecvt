#define POT_PIN 32 // measures at potentiometer slider
#define PWM_PIN 13 // SV on driver, sends pulses to the motor 
#define DIRECTION_PIN 26 // F/R on driver, controls the motor's direction
#define PRIMARY_HALL_PIN 14 // reads each time the hall sensor passes a magnet
#define WHEEL_HALL_PIN 39 // wheel speed hall sensor
#define ENCODER_A 27
#define ENCODER_B 14

#define DEBUG_RX 16 // UART RX for debugging
#define DEBUG_TX 17 // UART TX for debugging

// not used yet, input values as necessary 
// #define potPinTwo //measures across whole potentiometer resistor
// #define potPowerPin  //powers potentiometer
// #define brake_pin  //starts and stops the motor
// #define calibration_button  // starts calibration sequence
// #define limit_switch_one  //measures one of the maximum sheave travel positions
// #define limit_switch_one_power  //powers limit switch 1
// #define limit_switch_two  //measures the other maximum sheave travel position 
// #define limit_switch_two_power  //powers limit switch 2
// #define manualButtonOne  //starts manual mode 
// #define manualButtonTwo  //starts manual mode 
// #define buttonPower  // powers buttons for calibration and manual modes
// #define enablePin; // runs when high or floating 