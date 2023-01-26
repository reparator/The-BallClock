*****************************************************************
********************  Ball Clock Controller   *******************
*****************************************************************
* Controller for my BallClock (based on Arduino AT mega 328)
* Controlls 3 Stepper Motors 28BYJ-48 for Hours, Minutes and Seconds.
* @author: Stefan Kneip
* @date:   21.12.2022 (last touch)
*
* released under a creative commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
* https://creativecommons.org/licenses/by-nc-sa/4.0/
*
* Bounce2 library found here :
* // https://github.com/thomasfredericks/Bounce-Arduino-Wiring
*  I used version=2.53
*
* Adafruit RTC-Lib found here:
* // https://github.com/adafruit/RTClib
* I used version=1.8.0
* 
* My Stepper library:
* See attached folder /MyStepper
* 
*  Stepper 28BYJ-48
*  Stepper-Coil-Assignment:
*  Coil-Nr.	Color			JST-Nr.	Arduino-Pin		Shiftout-Pin
*  Coil 1	orange		    2      	    D2				Q1 / Q5
*  Coil 2	pink		    4			D4				Q3 / Q7
*  Coil 3	yellow		    3			D3				Q2 / Q6
*  Coil 4   blue			5			D5				Q4 / Q8
*  Common   red			    1			VCC				VCC
