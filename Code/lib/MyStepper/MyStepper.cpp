/* MyStepper.cpp - A Stepper Library for Arduino to control a Ministeppermotor 28BYJ-48
*
*  This library is designed to drive an unipolar geared stepper motor 28-BYJ-48
*  with ULN2803 as driver. Do not connect the motor directly to the arduino!
*  The high current will burn the arduino.
*  Another option is to connect the Stepper by a shift-register 74HC595. The advantage is,
*  that you need only 3 Pins for two steppers. Lower nibble (0-3) of the shift-register is
*  one stepper, higher nibble (4-7) of the shift-register is motor 2.
*
*  Copyright 2019 by Stefan Kneip
*  The library supports to ways of controlling, half-step and full-step:
*  The unipolar-motor has 5 wires. The red one is connect to VCC.
*  Stepper 28BYJ-48
*  Stepper-Coil-Assignment:
*  Coil-Nr.	Color			JST-Nr.	Arduino-Pin		Shiftout-Pin
*  Coil 1		orange		2      	D2								Q1 / Q5
*  Coil 2		pink			4				D4								Q3 / Q7
*  Coil 3		yellow		3				D3								Q2 / Q6
*  Coil 4    blue			5				D5								Q4 / Q8
*  Common    red			1				VCC								VCC
*
*  To control the motor half-step:
*		      Orange	Pink	 Gelb	   Blau
*  Step		Coil1	  Coil2	 Coil3	 Coil4
*	   1			0		   1	    0 		  1
*	   2			0	     1		  1		    0
*	   3			1		   0		  1		    0
*	   4			1		   0		  0		    1
*
*  To control the motor full-step:
*		      Orange	Pink	 Gelb	   Blau
*  Step		Coil1	  Coil2	 Coil3	 Coil4
*	   1			0		   1	    0 		  1
*	   2			0	     1		  0		    0
*	   3			0		   1		  1		    0
*	   4			0		   0		  1		    0
*    5      1      0      1       0
*    6      1      0      0       0
*    7      1      0      0       1
*    8      0      0      0       1
*/
#include "Arduino.h"
#include "MyStepper.h"
/* Constructor1: Sets the four wires to control the stepper */
MyStepper::MyStepper(int step_mode, int coil_1, int coil_2, int coil_3, int coil_4)
{
  this->con_mode = 0;												// control-Mode (0 = direct, 1 = Shift-register)
  // this->act_step = 0;												// actual step
  this->direction = 0;											// direction motor
  this->last_step_time = 0;									// time stamp of the last step
  //this->number_of_steps = 1000;	// number of steps in total
  this->step_mode = step_mode;							// step-Mode half (1) or full (2)
  this->step_delay = 2000L;                  // Pre-define the delay between two steps in mikrosec

  this->coil_1 = coil_1;
  this->coil_2 = coil_2;
  this->coil_3 = coil_3;
  this->coil_4 = coil_4;
  //Setup the pins
  pinMode(this->coil_1, OUTPUT);
  digitalWrite(this->coil_1, LOW);
  pinMode(this->coil_2, OUTPUT);
  digitalWrite(this->coil_2, LOW);
  pinMode(this->coil_3, OUTPUT);
  digitalWrite(this->coil_3, LOW);
  pinMode(this->coil_4, OUTPUT);
  digitalWrite(this->coil_4, LOW);
}
// Constructor2: three wires for shift register, four wires on the shift register for the stepper
MyStepper::MyStepper(int step_mode, int latch, int clock, int data, int coil_1, int coil_2, int coil_3, int coil_4){
  this->con_mode = 1;												// control-Mode (0 = direct, 1 = Shift-register)
  //this->act_step = 0;												// actual step
  this->direction = 0;											// direction motor
  this->last_step_time = 0;									// time stamp of the last step
  //this->number_of_steps = 1000;           	// number of steps in total
  this->step_mode = step_mode;							// step-Mode half (1) or full (2)
  this->step_delay = 2000L;                  // Pre-define the delay between two steps in mikrosec

  this->latch = latch;                      // Shiftregister LATCHPIN
  this->clock = clock;                      // Shiftregister CLOCKPIN
  this->data = data;                        // Shiftregister DATAPIN
  this->coil_1 = coil_1;                    // Output Shiftregister Coil1
  this->coil_2 = coil_2;                    // Output Shiftregister Coil2
  this->coil_3 = coil_3;                    // Output Shiftregister Coil3
  this->coil_4 = coil_4;                    // Output Shiftregister Coil4

  //Setup the pins
  pinMode(this->latch, OUTPUT);
  pinMode(this->clock, OUTPUT);
  pinMode(this->data, OUTPUT);
  pinMode(this->coil_1, OUTPUT);
  digitalWrite(this->coil_1, LOW);
  pinMode(this->coil_2, OUTPUT);
  digitalWrite(this->coil_2, LOW);
  pinMode(this->coil_3, OUTPUT);
  digitalWrite(this->coil_3, LOW);
  pinMode(this->coil_4, OUTPUT);
  digitalWrite(this->coil_4, LOW);
}
/* Sets the speed */
void MyStepper::setSpeed(int whatSpeed){
  // Erfahrungswerte: min. 3.000L, Maximum 20.000L bei Half-Step
  // (für step_delay) min. 2.000L, Maximum 20.000L bei Full-Step
  whatSpeed = constrain(whatSpeed, 0, 100);
  this->step_delay = map(long(whatSpeed), 100, 0, 1500, 15000);
  if (whatSpeed == 100){this->step_delay=2500L;};
}
void MyStepper::stopMotor(void){        //Stopps the motors and turns power off
  if (this->con_mode == 0){             //connected via Arduino-Pins
    stepPinOut(0, 0, 0, 0);
  }
  if (this->con_mode == 1){							//connected via Shiftregister
    stepShiftOut(0, 0, 0, 0);
  }
}
void MyStepper::step(int steps_to_move){
  int count;
  int steps_left = abs(steps_to_move);  // how many steps to take
  this->number_of_steps = steps_left;
  // determine direction based on whether steps_to_mode is + or -:
  if (steps_to_move > 0) { this->direction = 1; }
  if (steps_to_move < 0) { this->direction = 0; }
  // decrement the number of steps, moving one step each time:
  while (steps_left > 0){
    unsigned long now = micros();
    // move only if the appropriate delay has passed:
    if (now - this->last_step_time >= this->step_delay)
    {
      // get the timeStamp of when you stepped:
      this->last_step_time = now;
      // increment or decrement the step number,
      // depending on direction:
      if (this->direction == 1)
      {
        this->step_number++;
        if (this->step_number == this->number_of_steps) {
          this->step_number = 0;
        }
      }
      else
      {
        if (this->step_number == 0) {
          this->step_number = this->number_of_steps;
        }
        this->step_number--;
      }
      // decrement the steps left:
      steps_left--;
      if (this->step_mode == 1){
        count = this->step_number % 4;
      }
      if (this->step_mode == 2){
        count = this->step_number % 8;
      }
      stepMotor(count);
    }
  }
  stopMotor();
}
// Moves the motor forward or backwards//
void MyStepper::stepMotor(int thisStep){
  if (this->con_mode == 0){							//direct connected motor
      // Halfstep-Mode
    if (this->step_mode == 1) {
      switch (thisStep) {
        case 0:  // 0101
        stepPinOut(0, 1, 0, 1);
        break;
        case 1:  // 0110
        stepPinOut(0, 1, 1, 0);
        break;
        case 2:  // 1010
        stepPinOut(1, 0, 1, 0);
        break;
        case 3:  // 1001
        stepPinOut(1, 0, 0, 1);
        break;
      }
    }
    // Fullstep-Mode
    if (this->step_mode == 2) {
      switch (thisStep) {
        case 0:  // 0101
        stepPinOut(0, 1, 0, 1);
        break;
        case 1:  // 0100
        stepPinOut(0, 1, 0, 0);
        break;
        case 2:  // 0110
        stepPinOut(0, 1, 1, 0);
        break;
        case 3:  // 0010
        stepPinOut(0, 0, 1, 0);
        break;
        case 4:  // 1010
        stepPinOut(1, 0, 1, 0);
        break;
        case 5:  // 1000
        stepPinOut(1, 0, 0, 0);
        break;
        case 6:  // 1001
        stepPinOut(1, 0, 0, 1);
        break;
        case 7:  // 0001
        stepPinOut(0, 0, 0, 1);
        break;
      }
    }
  }
  if (this->con_mode == 1){							//connected via Shiftregister
      // Halfstep-Mode
    if (this->step_mode == 1) {
      switch (thisStep) {
        case 0:  // 0101
        stepShiftOut(0, 1, 0, 1);
        break;
        case 1:  // 0110
        stepShiftOut(0, 1, 1, 0);
        break;
        case 2:  // 1010
        stepShiftOut(1, 0, 1, 0);
        break;
        case 3:  // 1001
        stepShiftOut(1, 0, 0, 1);
        break;
      }
    }
    // Fullstep-Mode
    if (this->step_mode == 2) {
      switch (thisStep) {
        case 0:  // 0101
        stepShiftOut(0, 1, 0, 1);
        break;
        case 1:  // 0100
        stepShiftOut(0, 1, 0, 0);
        break;
        case 2:  // 0110
        stepShiftOut(0, 1, 1, 0);
        break;
        case 3:  // 0010
        stepShiftOut(0, 0, 1, 0);
        break;
        case 4:  // 1010
        stepShiftOut(1, 0, 1, 0);
        break;
        case 5:  // 1000
        stepShiftOut(1, 0, 0, 0);
        break;
        case 6:  // 1001
        stepShiftOut(1, 0, 0, 1);
        break;
        case 7:  // 0001
        stepShiftOut(0, 0, 0, 1);
        break;
      }
    }
  }
}
void MyStepper::stepPinOut(bool c1, bool c2, bool c3, bool c4){
  //* Perform a direct output to the Arduino-Pins
  digitalWrite(this->coil_1, c1);
  digitalWrite(this->coil_2, c2);
  digitalWrite(this->coil_3, c3);
  digitalWrite(this->coil_4, c4);
}
void MyStepper::stepShiftOut(bool c1, bool c2, bool c3, bool c4){
  //* Perform a output via Shiftregister-Pins
    digitalWrite(this->latch, LOW);
    byte pattern = 0b00000000;
    bitWrite(pattern, this->coil_1, c1);
    bitWrite(pattern, this->coil_2, c2);
    bitWrite(pattern, this->coil_3, c3);
    bitWrite(pattern, this->coil_4, c4);
    shiftOut(this->data, this->clock, MSBFIRST, pattern);
    digitalWrite(this->latch, HIGH);
}
/* version() returns the version of the library: */
int MyStepper::version(void){
  return 3;
}
