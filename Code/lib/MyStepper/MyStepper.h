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
*			Blau	Pink	Gelb	Orange
*  Step		Coil1	Coil2	Coil3	Coil4
*	1			1		1		0		0
*	2			0		1		1		0
*	3			0		0		1		1
*	4			1		0		0		1
*
*  To control the motor full-step:
*			Blau	Pink	Gelb	Orange
*  Step		Coil1	Coil2	Coil3	Coil4
*	1			1		1		0		0
*	2			0		1		0		0
*	3			0		1		1		0
*	4			0		0		1		0
*	5			0		0		1		1
*	6			0		0		0		1
*	7			1		0		0		1
*	8			1		0		0		0
*
*/
#ifndef MyStepper_h
#define MyStepper_h

// library interface description
class MyStepper {
	public:
	// constructors:
	// Connected via shiftregister 74HC595
	MyStepper(int step_mode, int latch, int clock, int data, int coil_1, int coil_2, int coil_3, int coil_4);
	// Connected via 4 direct Arduino pins
	MyStepper(int step_mode, int coil_1, int coil_2, int coil_3, int coil_4);

	void setSpeed(int Speed);					// Set the speed (0-100%)
	void stopMotor(void);								// stop the motor, turn power off
	void step(int number_of_steps);			//Move the Stepper
  int version(void);									// Version of this library

	private:
	void stepMotor(int this_step);
	void stepPinOut(bool c1, bool c2, bool c3, bool c4);
	void stepShiftOut(bool c1, bool c2, bool c3, bool c4);
	int con_mode;						     	// Connection mode: 0 = direct connected via Pin, 1 = connected via Shiftregister
	int direction;                // Direction of rotation (0-CW, 1=CCW)
	unsigned long step_delay;     // delay between steps, in ms, based on speed
	int number_of_steps;          // total number of steps this motor can take
	int step_mode;                // step-Mode half (1) or full (2)
	int step_number;              // which step the motor is on
	// motor coil numbers:
	int coil_1;
	int coil_2;
	int coil_3;
	int coil_4;
	int latch;							//LATCHPIN Shiftregister
	int clock;							//CLOCKPIN Shiftregister
	int data;								//DATAPIN Shiftregister

	unsigned long last_step_time; // time stamp in us of when the last step was taken
};

#endif
