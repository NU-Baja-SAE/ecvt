#include <Arduino.h>


// warning i do not know c/c++ however i do know arduino (ish) so that what i wrote this in but its taking issue with a lot of the things im 
//doing and i don't know if its a me issue or an ide-wants-c/c++ issue. nevertheless it is something. 


// E-cvt automatic mode skeleton code
// Nominal restricted Kohler idle is 1700rpm, max is 3800rpm, and peak power is at 3000rpm.
int idleRPM = 1800; // Threshold RPM for idle to engage
int setpointRPM = 3000; // Engine power-maximizing RPM, and target for PID
int numHallMagnets = 6; // Number of magnets per revolution that trigger hall sensor
int encoderResolution = 4095; // Default 12-bit esp adc resolution, for scaling purposes

// Positions range from 0 to 1000, and are scaled later to encoder input range.
int idlePos = 50; // Sheave position where sheave disengages for idle
int lowPos = 100; // Sheave position for lowest gear possible
int highPos = 900; // Sheave position for highest gear possible
int manualPosOne; 
int manualPosTwo; 


//ints for position pid gains;
//ints for engine rpm pid gains;
int engineRPM; // Calculated from hall sensor
int engineRPMavg; //rpm after multisampling
int numSamples = 0; // number of samples over which the average is taken
int encoderReading; // ADC input from linear potentiometer
float position; // Sheave position, scale it to between 0-1000.
int speed;
int hallCount = 0; // Hall sensor counts 
int rpmTimerDeltaT = 500; // How often the timer triggers, in seconds, should be faster than pid loop methinks 
int pidLoopDelay; // how often the timer triggers in us

uint16_t pot_v; // measurement across sliding part of potentiometer
uint16_t total_v = 4095; //measuremetn across potentiometer used as resistor, SET FOR TESTING
uint8_t positionZero; //zero value based on sheave travel (may not be full retraction of potentiometer)
uint8_t positionThousand; //1000 value based on sheave travel (may not be full extension of potentiometer)

uint8_t potPinOne = 32; // measures at potentiometer slider
uint8_t potPinTwo = 2; //measures across whole potentiometer resistor
uint8_t potPowerPin = 12; //powers potentiometer
uint8_t pwmPin = 33; // SV on driver, sends pulses to the motor 
uint8_t directionPin = 25; // F/R on driver, controls the motor's direction
uint8_t brakePin = 5; //starts and stops the motor
uint8_t hallOutputPin = 1; // reads each time the hall sensor passes a magnet
uint8_t calibrationButton = 7; // starts calibration sequence
uint8_t limitSwitchOne = 8; //measures one of the maximum sheave travel positions
uint8_t limitSwitchOnePower = 13; //powers limit switch 1
uint8_t limitSwitchTwo = 9; //measures the other maximum sheave travel position 
uint8_t limitSwitchTwoPower = 14; //powers limit switch 2
uint8_t manualButtonOne = 10; //starts manual mode 
uint8_t manualButtonTwo = 11; //starts manual mode 
uint8_t buttonPower = 15; // powers buttons for calibration and manual modes
uint8_t enablePin; // runs when high or floating 


int setpointPos;//determined by engine rpm
// pid loop coefficients
int kpPos = 1;
int kiPos = 1;
int kdPos = 1;

int kpRPM = 1;
int kiRPM = 1;
int kdRPM = 1; 

float lastErrorPos = 0;
float errorPos = 0;
float changeErrorPos = 0;
float totalErrorPos = 0;
float pidTermPos = 0;

float lastErrorRPM = 0;
float errorRPM = 0;
float changeErrorRPM = 0;
float totalErrorRPM = 0;
float pidTermRPM = 0;
float threshold = 2000;
bool hall_trigger;  
  
// tested code:  
//   position = analogRead(potPinOne) * 256/4096; // doesn't really matter but division by multiples of 2 is much faster

//   if (position < 128){ // 127 should be 0, 0 should be 255
//     digitalWrite(directionPin, HIGH);
//     position = (127 - position) * 2;
//   }
//   else { // 128 should be 0, 255 should be 255
//     digitalWrite(directionPin, LOW);
//     position = (position - 128) * 2;
//   }

//   analogWrite(pwmPin, position);
//   delay(10);


// selections from main code that control direction switching (do they look like they would work?)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(potPinOne, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pot_v = analogRead(potPinOne);
}

void loop() {
    while (pot_v < 220){
        Serial.println(pot_v);
        digitalWrite(directionPin, HIGH);
        analogWrite(pwmPin, 127);
        pot_v = analogRead(potPinOne);
    }

    analogWrite(pwmPin, 0);
    delay(1000);

    while (pot_v > 30){
        Serial.println(pot_v);
        digitalWrite(directionPin, LOW);
        analogWrite(pwmPin, 127);
        pot_v = analogRead(potPinOne);
    }

    analogWrite(pwmPin, 0);
    delay(1000);
}




#include <Arduino.h>


// warning i do not know c/c++ however i do know arduino (ish) so that what i wrote this in but its taking issue with a lot of the things im 
//doing and i don't know if its a me issue or an ide-wants-c/c++ issue. nevertheless it is something. 


// E-cvt automatic mode skeleton code
// Nominal restricted Kohler idle is 1700rpm, max is 3800rpm, and peak power is at 3000rpm.
int idleRPM = 1800; // Threshold RPM for idle to engage
int setpointRPM = 3000; // Engine power-maximizing RPM, and target for PID
int numHallMagnets = 6; // Number of magnets per revolution that trigger hall sensor
int encoderResolution = 4095; // Default 12-bit esp adc resolution, for scaling purposes

// Positions range from 0 to 1000, and are scaled later to encoder input range.
int idlePos = 50; // Sheave position where sheave disengages for idle
int lowPos = 100; // Sheave position for lowest gear possible
int highPos = 900; // Sheave position for highest gear possible
int manualPosOne; 
int manualPosTwo; 


//ints for position pid gains;
//ints for engine rpm pid gains;
int engineRPM; // Calculated from hall sensor
int engineRPMavg; //rpm after multisampling
int numSamples = 0; // number of samples over which the average is taken
int encoderReading; // ADC input from linear potentiometer
float position; // Sheave position, scale it to between 0-1000.
int speed;
int hallCount = 0; // Hall sensor counts 
int rpmTimerDeltaT = 500; // How often the timer triggers, in seconds, should be faster than pid loop methinks 
int pidLoopDelay; // how often the timer triggers in us

uint16_t pot_v; // measurement across sliding part of potentiometer
uint16_t total_v = 4095; //measuremetn across potentiometer used as resistor, SET FOR TESTING
uint8_t positionZero; //zero value based on sheave travel (may not be full retraction of potentiometer)
uint8_t positionThousand; //1000 value based on sheave travel (may not be full extension of potentiometer)

uint8_t potPinOne = 32; // measures at potentiometer slider
uint8_t potPinTwo = 2; //measures across whole potentiometer resistor
uint8_t potPowerPin = 12; //powers potentiometer
uint8_t pwmPin = 33; // SV on driver, sends pulses to the motor 
uint8_t directionPin = 25; // F/R on driver, controls the motor's direction
uint8_t brakePin = 5; //starts and stops the motor
uint8_t hallOutputPin = 1; // reads each time the hall sensor passes a magnet
uint8_t calibrationButton = 7; // starts calibration sequence
uint8_t limitSwitchOne = 8; //measures one of the maximum sheave travel positions
uint8_t limitSwitchOnePower = 13; //powers limit switch 1
uint8_t limitSwitchTwo = 9; //measures the other maximum sheave travel position 
uint8_t limitSwitchTwoPower = 14; //powers limit switch 2
uint8_t manualButtonOne = 10; //starts manual mode 
uint8_t manualButtonTwo = 11; //starts manual mode 
uint8_t buttonPower = 15; // powers buttons for calibration and manual modes
uint8_t enablePin; // runs when high or floating 


int setpointPos;//determined by engine rpm
// pid loop coefficients
int kpPos = 1;
int kiPos = 1;
int kdPos = 1;

int kpRPM = 1;
int kiRPM = 1;
int kdRPM = 1; 

float lastErrorPos = 0;
float errorPos = 0;
float changeErrorPos = 0;
float totalErrorPos = 0;
float pidTermPos = 0;

float lastErrorRPM = 0;
float errorRPM = 0;
float changeErrorRPM = 0;
float totalErrorRPM = 0;
float pidTermRPM = 0;
float threshold = 2000;
bool hall_trigger;  
  
// tested code:  
//   position = analogRead(potPinOne) * 256/4096; // doesn't really matter but division by multiples of 2 is much faster

//   if (position < 128){ // 127 should be 0, 0 should be 255
//     digitalWrite(directionPin, HIGH);
//     position = (127 - position) * 2;
//   }
//   else { // 128 should be 0, 255 should be 255
//     digitalWrite(directionPin, LOW);
//     position = (position - 128) * 2;
//   }

//   analogWrite(pwmPin, position);
//   delay(10);


// selections from main code that control direction switching (do they look like they would work?)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(potPinOne, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pot_v = analogRead(potPinOne);
}

void loop() {
    while (pot_v < 3900){
        Serial.println(pot_v);
        Serial.println("Increasing");
        digitalWrite(directionPin, HIGH);
        analogWrite(pwmPin, 50);
        pot_v = analogRead(potPinOne);
    }

    analogWrite(pwmPin, 0);
    delay(1000);

    while (pot_v > 300){
        Serial.println(pot_v);
        Serial.println("Decreasing");
        digitalWrite(directionPin, LOW);
        analogWrite(pwmPin, 50);
        pot_v = analogRead(potPinOne);
    }

    analogWrite(pwmPin, 0);
    delay(1000);
}