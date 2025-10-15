#include <Arduino.h>
#include "motor.h"
#include "hall_sensor.h"
#include "potentiometer.h"
#include "pins.h"


// warning i do not know c/c++ however i do know arduino (ish) so that what i wrote this in but its taking issue with a lot of the things im 
//doing and i don't know if its a me issue or an ide-wants-c/c++ issue. nevertheless it is something. 


// E-cvt automatic mode skeleton code
// Nominal restricted Kohler idle is 1700rpm, max is 3800rpm, and peak power is at 3000rpm.

// Positions range from 0 to 1000, and are scaled later to encoder input range.

int manualPosOne; 
int manualPosTwo; 


//ints for position pid gains;
//ints for engine rpm pid gains;
int engine_RPM; // Calculated from hall sensor
// int engineRPMavg; //rpm after multisampling
// int numSamples = 0; // number of samples over which the average is taken
// int encoderReading; // ADC input from linear potentiometer
float position; // Sheave position, scale it to between 0-1000.
int speed; 

int pid_loop_delay = 100000; // how often the timer triggers in us

uint16_t pot_v; // measurement across sliding part of potentiometer
uint16_t total_v; //measuremetn across potentiometer used as resistor
uint8_t positionZero; //zero value based on sheave travel (may not be full retraction of potentiometer)
uint8_t positionThousand; //1000 value based on sheave travel (may not be full extension of potentiometer)



// not used yet 
uint8_t potPinTwo = 2; //measures across whole potentiometer resistor
uint8_t potPowerPin = 12; //powers potentiometer
uint8_t brake_pin = 5; //starts and stops the motor
uint8_t calibration_button = 7; // starts calibration sequence
uint8_t limit_switch_one = 8; //measures one of the maximum sheave travel positions
uint8_t limit_switch_one_power = 13; //powers limit switch 1
uint8_t limit_switch_two = 9; //measures the other maximum sheave travel position 
uint8_t limit_switch_two_power = 14; //powers limit switch 2
uint8_t manualButtonOne = 10; //starts manual mode 
uint8_t manualButtonTwo = 11; //starts manual mode 
uint8_t buttonPower = 15; // powers buttons for calibration and manual modes
uint8_t enablePin; // runs when high or floating 


int setpoint_pos;//determined by engine rpm
// pid loop coefficients
int kp_pos = 1;
int ki_pos = 1;
int kd_pos = 1;

int kp_RPM = 1;
int ki_RPM = 1;
int kd_RPM = 1; 

float last_error_pos = 0;
float error_pos = 0;
float change_error_pos = 0;
float total_error_pos = 0;
float pid_term_pos = 0;

float last_error_RPM = 0;
float error_RPM = 0;
float change_error_RPM = 0;
float total_error_RPM = 0;
float pid_term_RPM = 0;
 
int v;

// if ypu dont get the naming conventions of my timer variables i don't either it's just the same conventions ilya uses 
// and in ilya we trust prayer hands emoji x3

// i dont really get the timer either so anyone reading this should expect Many Issues

hw_timer_t * pid_timer_pos = NULL; 
portMUX_TYPE pid_timer_pos_MUX = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t * pid_timer_RPM = NULL; 
portMUX_TYPE pid_timer_RPM_MUX = portMUX_INITIALIZER_UNLOCKED;


void IRAM_ATTR pid_loop_RPM() {
  portENTER_CRITICAL_ISR(&pid_timer_RPM_MUX);
    
    //setpoint = 3000
    //current = engineRPM

    rpm_pid_calc();
  portEXIT_CRITICAL_ISR(&pid_timer_RPM_MUX);
}

 
void IRAM_ATTR pid_loop_pos() {
  portENTER_CRITICAL_ISR(&pid_timer_pos_MUX);
    //setpoint = pidTermRPM
    //current = position
    read_pos(); //find the current sheave position scaled 1-1000

    //constrain travel before limit switches 
    if(position > HIGH_POS) {
      setpoint_pos = HIGH_POS; 
      //setDirection();
      pos_pid_calc();
      //ledcWrite(pwmPin, pidTermPos);
      set_direction_speed(pid_term_pos);
    }
    else if(position<LOW_POS){
      setpoint_pos = LOW_POS; 
      //setDirection();
      pos_pid_calc();
      //ledcWrite(pwmPin, pidTermPos);
      set_direction_speed(pid_term_pos);
    }
    else if(engine_RPM < IDLE_RPM){
      setpoint_pos = IDLE_POS; 
      //setDirection();
      pos_pid_calc(); 
      //ledcWrite(pwmPin, pidTermPos);
      set_direction_speed(pid_term_pos);
    }
    // unconstrained travel
    else{
      setpoint_pos = pid_term_RPM; 
      //setDirection();
      pos_pid_calc();
      //ledcWrite(pwmPin, pidTermPos);
      set_direction_speed(pid_term_pos);
    }

  portEXIT_CRITICAL_ISR(&pid_timer_pos_MUX);
}

//add multisampling tiemr 


// put function declarations here:

// Interrupt to count hall effect rising edges (each time a magnet passes the sensor)
// Interrupt to count hall effect rising edges (each time a magnet passes the sensor)
hw_timer_t * hall_timer = NULL; 
portMUX_TYPE hall_timer_MUX = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR hall_interrupt() {
  portENTER_CRITICAL_ISR(&hall_timer_MUX);
    v = analogRead(HALL_OUTPUT_PIN);
    read_hall(v);

  portEXIT_CRITICAL_ISR(&hall_timer_MUX);
}

// interrupt to run calibration sequence 
void IRAM_ATTR calibration_interrupt(){

while(bool pressed = LOW) {
  pressed = digitalRead(limit_switch_one);
  digitalWrite(DIRECTION_PIN, HIGH);
  analogWrite(PWM_PIN, 50);
}
// modify this -- are wea reading across the whole thing bc idk if we're doing the scaling thing anymore
 pot_v = analogRead(POT_PIN); // potentiometer slider

 positionZero = pot_v; 


while(bool pressed = LOW) {
  pressed = digitalRead(limit_switch_two);
  digitalWrite(DIRECTION_PIN, LOW);
  analogWrite(PWM_PIN, 50);
}

 pot_v = analogRead(POT_PIN); // potentiometer slider

 positionThousand = 1000 / pot_v; 

}


void IRAM_ATTR end_case_one() {
  setpoint_pos = HIGH_POS; 
  pos_pid_calc(); 
  set_direction_speed(pid_term_pos);

}

void IRAM_ATTR end_case_two() {
  setpoint_pos = LOW_POS; 
  pos_pid_calc(); 
  set_direction_speed(pid_term_pos);

}

//hold down button to keep mode on 
void IRAM_ATTR manual_mode_one() {
  bool pressed = digitalRead(manualButtonOne);
  while(pressed = HIGH) {
  pressed = digitalRead(manualButtonOne);
  setpoint_pos = manualPosOne; 
  pos_pid_calc();
  set_direction_speed(pid_term_pos);
  }
}

void IRAM_ATTR manual_mode_two() {
  bool pressed = digitalRead(manualButtonTwo);
  while(pressed = HIGH) {
  setpoint_pos = manualPosTwo;
  pos_pid_calc();
  set_direction_speed(pid_term_pos);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  

  //configure whether pins will be sending or receiving data
  setup_motor();
  setup_hall();
  setup_potentiometer();
  pinMode(potPinTwo, INPUT);
  pinMode(potPowerPin, OUTPUT);

  pinMode(brake_pin, OUTPUT);

  pinMode(calibration_button, INPUT);
  pinMode(limit_switch_one, INPUT);
  pinMode(limit_switch_one_power, OUTPUT);
  pinMode(limit_switch_two, INPUT);
  pinMode(limit_switch_two_power, OUTPUT);
  pinMode(manualButtonOne, INPUT); 
  pinMode(manualButtonTwo, INPUT); 
  pinMode(buttonPower, OUTPUT);

  //initialize variable values, will def need to add more
  digitalWrite(brake_pin, HIGH); // set brake off
 
  digitalWrite(potPowerPin, HIGH); // power potentiometer at voltage that wont break MCU (3.3V)
  digitalWrite(limit_switch_one_power, HIGH);
  digitalWrite(limit_switch_two_power, HIGH);
  digitalWrite(buttonPower, HIGH); 

  // interrupt set up
  attachInterrupt(digitalPinToInterrupt(calibration_button), calibration_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(limit_switch_one), end_case_one, RISING);
  attachInterrupt(digitalPinToInterrupt(limit_switch_two), end_case_two, RISING);

  //hardware timer setup
  hall_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(hall_timer, &hall_interrupt, true);
  timerAlarmWrite(hall_timer, HALL_TIMER_DELAY, true);
  timerAlarmEnable(hall_timer);


  pid_timer_RPM = timerBegin(2, 80, true);
  timerAttachInterrupt(pid_timer_RPM, &pid_loop_RPM, true);
  timerAlarmWrite(pid_timer_RPM, pid_loop_delay, true);
  timerAlarmEnable(pid_timer_RPM);

  pid_timer_pos = timerBegin(3, 80, true);
  timerAttachInterrupt(pid_timer_pos, &pid_loop_pos, true);
  timerAlarmWrite(pid_timer_pos, pid_loop_delay, true);
  timerAlarmEnable(pid_timer_pos);

}

void loop() {
  // put your main code here, to run repeatedly:

   
}

// put function definitions here:




void rpm_pid_calc() {
error_RPM = SETPOINT_RPM - rpm_calc();

change_error_RPM = error_RPM - last_error_RPM;
total_error_RPM += error_RPM;
pid_term_RPM = (kp_RPM * error_RPM) + (ki_RPM * total_error_RPM) + (kd_RPM * change_error_RPM);

last_error_RPM = error_RPM;

}

void pos_pid_calc() {
  error_pos = setpoint_pos - position;

  change_error_pos = error_pos - last_error_pos; 
  total_error_pos += error_pos;
  pid_term_pos = (kp_pos * error_pos) + (ki_pos * total_error_pos) + (kd_pos * change_error_pos);

  last_error_pos = error_pos; 
}



