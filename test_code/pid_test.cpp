#include <Arduino.h>
#include "motor.h"

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
int highPos = 4000; // Sheave position for highest gear possible
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
int pidLoopDelay = 100000; // how often the timer triggers in us

uint16_t pot_v; // measurement across sliding part of potentiometer
uint16_t total_v; //measuremetn across potentiometer used as resistor
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
int kpPos = 0.5;
int kiPos = 0;
int kdPos = 0;

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
  


// selections from main code that control direction switching (do they look like they would work?)

 void readPos() {
    pot_v = analogRead(potPinOne); // potentiometer lever
// total_v = analogRead(potPinTwo); // whole resistor

// read the voltage across the potentiometer as a proportion, because when the voltage changes as the battery drains, the proportion will remain constant 
   // encoderReading = pot_v / total_v; // find how far along the resistor the potentiometer is as a fraction
    position = pot_v; // scale to make more workable number based on zero/thousand setpoints from calibration
    Serial.println("position read");
}


 void posPidCalc() {
  errorPos = position - setpointPos;

  changeErrorPos = errorPos - lastErrorPos; 
  totalErrorPos += errorPos;
  pidTermPos = (kpPos * errorPos) + (kiPos * totalErrorPos) + (kdPos * changeErrorPos);

  lastErrorPos = errorPos; 
}


hw_timer_t * pidTimerPos = NULL; 
portMUX_TYPE pidTimerPosMUX = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR pidLoopPos() {
  portENTER_CRITICAL_ISR(&pidTimerPosMUX);
    // setpoint = pidTermRPM
    // current = position
    readPos(); //find the current sheave position scaled 1-4095

    //constrain travel before limit switches 

    // this is less important for what i am lookign at rn 
    // if(position > highPos) {
    //   setpointPos = highPos; 
    //   setDirection();
    //   posPidCalc();
    //   ledcWrite(pwmPin, pidTermPos);
    // }
    // else if(position<lowPos){
    //   setpointPos = lowPos; 
    //   setDirection();
    //   posPidCalc();
    //   ledcWrite(pwmPin, pidTermPos);
    // }
    // else if(engineRPM < idleRPM){
    //   setpointPos = idlePos; 
    //   setDirection();
    //   posPidCalc(); 
    //   ledcWrite(pwmPin, pidTermPos);
    // }


    // unconstrained travel
    // this is VVVVVV important for what i'm looking at rn!!!!!
   // else{
      setpointPos = 2048; 
      // setDirection();
      posPidCalc();
     // ledcWrite(pwmPin, pidTermPos);
      set_direction_speed(pidTermPos);
      Serial.println(pidTermPos);
      Serial.println(errorPos);
   // }

    
  portEXIT_CRITICAL_ISR(&pidTimerPosMUX);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(potPinOne, INPUT);


  setup_motor();
  Serial.println("motor setup complete");

//   pidTimerPos = timerBegin(3, 80, true);
//   timerAttachInterrupt(pidTimerPos, &pidLoopPos, true);
//   timerAlarmWrite(pidTimerPos, pidLoopDelay, true);
//   timerAlarmEnable(pidTimerPos);
  Serial.println("timer setup complete");

  
}





// task: which direction is which? --> moving away from motor is positive (position is less than setpoint)
// fully retracted is position = 255 and motor screwing in (sheaves traveling outwards) and directionPin LOW
// also task: which position do we want to be position zero (eg fully together vs fully apart)?
// more task: make sure main code matches whatever we decide on for directional conventions 
// also do we need to break before changing directions? like should we add a if current_direction != next_direction type of situation 
// do we need the enable pin?
// //void setDirection() {
  
//   if (position < setpointPos){
//     digitalWrite(directionPin, HIGH);
//   }
//   else {
//     digitalWrite(directionPin, LOW);
//   }

//}



// tested direction switchin code in cole's test setup:

 void loop() {
//     while (pot_v < 3900){
//         Serial.println(pot_v);
//         Serial.println("Increasing");
//         digitalWrite(directionPin, HIGH);
//         analogWrite(pwmPin, 50);
//         pot_v = analogRead(potPinOne);
//     }

//     analogWrite(pwmPin, 0);
//     delay(1000);

//     while (pot_v > 300){
//         Serial.println(pot_v);
//         Serial.println("Decreasing");
//         digitalWrite(directionPin, LOW);
//         analogWrite(pwmPin, 50);
//         pot_v = analogRead(potPinOne);
//     }

//     analogWrite(pwmPin, 0);
    delay(100);
    readPos();
    Serial.printf("position: %d \n", position);
    Serial.printf("pid term : %d \n", pidTermPos);
    Serial.printf("error term : %d \n", errorPos);
 }

