/*****************************************************************
********************  Ball Clock Controller   *******************
*****************************************************************
* Controller for my BallClock (based on Arduino AT mega 328)
* Controlls 3 Stepper Motors 28BYJ-48 for Hours, Minutes and Seconds.
* @author: Stefan Kneip
* @date:   21.12.2022 (last touch)

* released under a creative commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
* https://creativecommons.org/licenses/by-nc-sa/4.0/
*
* Bounce2 library found here :
* // https://github.com/thomasfredericks/Bounce-Arduino-Wiring

*  Stepper 28BYJ-48
*  Stepper-Coil-Assignment:
*  Coil-Nr.	Color			JST-Nr.	Arduino-Pin		Shiftout-Pin
*  Coil 1	orange		    2      	    D2				Q1 / Q5
*  Coil 2	pink		    4			D4				Q3 / Q7
*  Coil 3	yellow		    3			D3				Q2 / Q6
*  Coil 4   blue			5			D5				Q4 / Q8
*  Common   red			    1			VCC				VCC
*/

#include <Arduino.h>
#include "RTClib.h"         // Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include <Bounce2.h>        // DeBounce library 
#include <MyStepper.h>      // My Stepper routines
#include <EEPROM.h>

//Pin Assignement
#define S1COIL1PIN           2  // SecondsStepper direct Coil1 (orange)
#define S1COIL2PIN           3  // SecondsStepper direct Coil2 (pink)
#define S1COIL3PIN           4  // SecondsStepper direct Coil3 (yellow)
#define S1COIL4PIN           5  // SecondsStepper direct Coil4 (blue)
#define CLOCKPIN             6  // Shift-Register 74HC595 SH_CP (CLOCKPIN)
#define LATCHPIN             7  // Shift-Register 74HC595 ST_CP (LATCHPIN)
#define DATAPIN              8  // Shift-Register 74HC595 DS (DATAPIN)
#define SET_PUSHBUTTONPIN    9  // Switch seconds ON / OFF   (Gelbe Taste)
#define R_PUSHBUTTONPIN     10  // Pushbutton Calibration    (Weisse Taste)
#define M_PUSHBUTTONPIN     11  // Pushbutton Minutes        (Schwarze Taste)
#define SET_LEDPIN          12  // Settings-Led
#define LEDBUILDINPIN       13  // ARDUINOLED
#define HALLSENS3PIN        A0  // Hall-Sensor Hours
#define HALLSENS2PIN        A1  // Hall-Sensor Minutes
#define HALLSENS1PIN        A2  // Hall-Sensor Seconds
#define H_PUSHBUTTONPIN     A3  // Set the hours                (Rote Taste)

#define DEBOUNCE            10  // button debouncer interval in ms
#define DEBOUNCEHALL         3  // debouncer for Hall-Sensors

#define STEPPERSEC           0  // No. of Steppers
#define STEPPERMIN           1
#define STEPPERHOUR          2

/* Die Carbonplatte hat eine Maserung. Damit diese beim Aufhängen oder Aufstellen schön waagerecht ist,
die Hall-Sensoren aber nicht genau oben sind, muss dieser Offset weitergedreht werden, damit die Kugeln
genau auf der 12 landen.(Carbonpatternoffset für alle Kugeln)
*/
// Offsets für 12 Uhr:
#define CARBONPATTERNOFFSET_SEC   0              // 4090
#define CARBONPATTERNOFFSET_MIN   0              // 7100
#define CARBONPATTERNOFFSET_HOUR  0              // 4600

// Direction of the Steppers
#define SEC_DIR         1                 // 1: Clockwise -1: CounterClockwise
#define MIN_DIR        -1                 // 1: Clockwise -1: CounterClockwise
#define HOUR_DIR       -1                 // 1: Clockwise -1: CounterClockwise

#define ROUND_SEC    10624      //10688         //Impulses for one round seconds 13952
#define ROUND_MIN     8096      //8256          //Impulses for one round minutes 22528
#define ROUND_HOUR    4704      //5184          //Impulses for one round hours 15232

#define BACKCORR    0.88                       // Korrigiert die Fehler beim Rückwärtsfahren

//  Blink-LED (Heartbeat)
boolean blinkFlag = false;      // help variable for heartbeat-mode (PIR-Modus)
boolean heartbeatFlag = true;   // true, if heartbeat is shown
int blinkDelayOn = 100;         // ON-time of the heartbeat-LED
int blinkDelayOff = 1000;       // OFF-time of the heartbeat-LED
unsigned long blinkDelay;       // variable for the blink delay
unsigned long blinkTime;        // helper variable to control the heartbeat-LED

// Button Control
boolean minPressed = false;     // true if button Min pressed
boolean hourPressed = false;    // true if button Hour pressed
boolean setPressed = false;     // true if button Set pressed
boolean resPressed = false;     // true if button Res pressed

// Hall Sensors
boolean hallDetect[3] = {false,false,false};        // true, if hall-Detector triggert

// TimerControl (time loops / time schedule)
unsigned long Sekundentakt = 100;       // 100 = every 1/10 second
unsigned long Minutentakt = 10000;      // 10000 = every 10 seconds
unsigned long Mikrotakt = 1;            // every step    
unsigned long Sekundenstart;            // starting point seconds
unsigned long Minutenstart;             // starting point minutes
unsigned long Mikrostart;               // micro starting point

byte sekunde;                           // helper variables
byte minute;
byte stunde;

byte lastSec;                           // helper variables
byte lastMin;
byte lastHour;

byte jahr;                              // helper variables
byte monat;
byte tag;

// Serial Receive Data (console input)
const byte numChars = 20;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;
int steps;
// int akt_stepper = 0;

// Stepper Control
// int number_of_steps = 4096; //Anzahl auszuführender Steps
byte step_mode = 1; // Stepmode: 1-halfstep, 2-fullstep
int stepper_speed = 80; //Speed of the Stepping (Min - 100 %)
// New Stepper-Instance: COIL1 bis Coil4 direkt über Arduino-Pins angesteuert
MyStepper stepperSec(1, S1COIL1PIN, S1COIL3PIN, S1COIL2PIN, S1COIL4PIN);
// New Stepper-Instance: Stepper an Ausgang 1 - 4 des Shift-Registers
MyStepper stepperMin(step_mode, LATCHPIN, CLOCKPIN, DATAPIN, 0, 2, 1, 3);
// New Stepper-Instance: Stepper an Ausgang 5 - 8 des Shift-Registers
MyStepper stepperHour(step_mode, LATCHPIN, CLOCKPIN, DATAPIN, 4, 6, 5, 7);
//MyStepper	stepperHour(step_mode, LATCHPIN, CLOCKPIN, DATAPIN, 5, 7, 4, 6);

// Realtime Clock RTC Dallas DS1307
RTC_DS1307 rtc; // Initiate an RTC instance

// Button debouncing
Bounce setButton = Bounce(); // Initiate the Bounce objects for the Pushbuttons
Bounce hourButton = Bounce();
Bounce minButton = Bounce();
Bounce resButton = Bounce();

// Hall Sensors debouncing
Bounce secHall = Bounce();
Bounce minHall = Bounce();
Bounce hourHall = Bounce();

//Clock things (for each stepper)
byte increment[3] = {64,32,32};    // No of steps peformed during one invoke of stepper_update (portion: 64 or 32)
boolean dirFlag[3] = {true,true,true};  //true = forward, false = backwards
int aktPos[3];     // actual positions on the wheel (0 - SEC, 1 - MIN, 2 - HOUR)
int toGoSteps[3];  // steps to perform before the target is reached
int fullRotation[3] = {ROUND_SEC,ROUND_MIN, ROUND_HOUR};     // predefined steps for one rotation (only for initialization, 
                                                             // will be calibrated later)
// Offsets between sensor trigger point and 12 o`clock
int offset[3] = {CARBONPATTERNOFFSET_SEC, CARBONPATTERNOFFSET_MIN, CARBONPATTERNOFFSET_HOUR}; 

int pos[3];
boolean secondsFlag = false; // true: second ball used, seconds updated, false: seconds switched off
boolean calibrationFlag = true;  //true: Calibration OK, false: Calibration necessary
                                 // set this flag to false the first time you upload the sketch to init a calibration
// For Calibration:
int secondSteps = 0;
int minuteSteps = 0;
int hourSteps = 0;

unsigned long calstarttime;
#define MAXCALTIME = 50000          // Calibrierung darf maximal 5 min dauern

// Step-Counter helpers
int helpSeconds=0;
int helpMinutes=0;
int helpHours=0;

boolean onFlag = false;             // true: output to the steppers, false: steppers switched off (for testing only)
boolean setupFlag = true;           // true: no automatic updates, false: update the steppers as requested

#define INITHOUR        5           // Time when daily initialisation starts (6 und 18 Uhr)
#define INITMIN         59

// EEPROM-Settings
int address = 0;                    // First EEPROM-Address
/* Daten:
    0       Fullrotation[2] Hours
    2       Fullrotation[1] Minutes
    4       Fullrotation[0] Seconds
    6       Offset[2] Hours
    8       Offset[1] Minutes
   10       Offset[0] Seconds
   12       SecondsFlag
*/
//Funktionsdeklarationen:
void interrupt_handling();
void printTime();
void printDate();
void heartbeat(int heartbeatPin);
void checkButtons();
void displayZeit(byte hour, byte minutes);
void checkHallSensors();
void checkInit();
void binaryOutput(int var);
int setZero(uint8_t stepper);
void recvData();
void computeData();
void list_commands();
void moveSteps(uint8_t stepper, int steps);
void calibration();
void parseTime();
void updateSteppers();
void time2Steps();
void initialize();
void EEpromWriteByte(int address, byte value);
void EEpromWriteInt(int address, int value);
int EEpromReadInt(int address);
boolean inc2limits(byte zahl, byte minimum, byte maximum, byte& ret);
boolean dec2limits(byte zahl, byte minimum, byte maximum, byte& ret);
boolean inRange (int value, int minvalue, int maxvalue);
int getIntFromString (char *stringWithInt, byte num);
//*******************************************************//
void setup()
{
    Serial.begin(57600); // Initialize Serial Communication
    //Setup the defined pins
    //Stepper for the Seconds
    pinMode(S1COIL1PIN, OUTPUT); // Coil1 Stepper for the seconds
    digitalWrite(S1COIL1PIN, LOW);
    pinMode(S1COIL2PIN, OUTPUT); // Coil2 Stepper for the seconds
    digitalWrite(S1COIL2PIN, LOW);
    pinMode(S1COIL3PIN, OUTPUT); // Coil3 Stepper for the seconds
    digitalWrite(S1COIL3PIN, LOW);
    pinMode(S1COIL4PIN, OUTPUT); // Coil4 Stepper for the seconds
    digitalWrite(S1COIL4PIN, LOW);
    // Shiftregister-Setup
    pinMode(LATCHPIN, OUTPUT); // Shift-Register Latch
    digitalWrite(LATCHPIN, LOW);
    pinMode(CLOCKPIN, OUTPUT); // Shift-Register Clock
    digitalWrite(CLOCKPIN, LOW);
    pinMode(DATAPIN, OUTPUT); // Shift-Register Data
    digitalWrite(DATAPIN, LOW);
    // LEDs
    pinMode(SET_LEDPIN, OUTPUT); // LED for Settings-Mode
    digitalWrite(SET_LEDPIN, LOW);
    pinMode(LEDBUILDINPIN, OUTPUT); // Buildin LED
    //Pushbuttons
    pinMode(H_PUSHBUTTONPIN, INPUT);    // Pushbutton to set hours
    pinMode(M_PUSHBUTTONPIN, INPUT);    // Pushbutton to set minutes
    pinMode(SET_PUSHBUTTONPIN, INPUT);  // Pushbutton to enter calibration-mode
    pinMode(R_PUSHBUTTONPIN, INPUT);    // Pushbutton to toggle seconds
    // Hall-Sensors
    pinMode(HALLSENS1PIN, INPUT);       // Hall-Sensor Seconds
    pinMode(HALLSENS2PIN, INPUT);       // Hall-Sensor Minutes
    pinMode(HALLSENS3PIN, INPUT);       // Hall-Sensor Hours
    pinMode(LEDBUILDINPIN, OUTPUT);     // Buildin LED

    Serial.println(F("The BallClock"));     // Communication ok!
    if (!rtc.begin()) {
        Serial.println(F("Couldn't find RTC"));
        while (1);
    }
    delay(500);
    if (!rtc.isrunning()) {
        Serial.println(F("RTC is NOT running!"));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    } else {
        Serial.println(F("RTC is running OK"));
        printTime();
    }
    if ((rtc.now().unixtime()) < (DateTime(__DATE__, __TIME__).unixtime())){
        Serial.print(F("Adjusting RTC"));
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    // Setup the Bounce instances :
    setButton.attach(SET_PUSHBUTTONPIN);
    setButton.interval(DEBOUNCE); // debounce interval in ms
    hourButton.attach(H_PUSHBUTTONPIN);
    hourButton.interval(DEBOUNCE);
    minButton.attach(M_PUSHBUTTONPIN);
    minButton.interval(DEBOUNCE);
    resButton.attach(R_PUSHBUTTONPIN);
    resButton.interval(DEBOUNCE);
    //Hall Sensors
    secHall.attach(HALLSENS1PIN);
    secHall.interval(DEBOUNCEHALL);
    minHall.attach(HALLSENS2PIN);
    minHall.interval(DEBOUNCEHALL);
    hourHall.attach(HALLSENS3PIN);
    hourHall.interval(DEBOUNCEHALL);
    //Steppers
    stepperSec.setSpeed(100);
    stepperMin.setSpeed(stepper_speed);
    stepperHour.setSpeed(stepper_speed);

    Serial.print(F("Stepper-Version: "));
    Serial.println(stepperSec.version());
     // read values for full rotation from EEPROM
    if ((EEpromReadInt(address) != int(0xFFFF)) && (EEpromReadInt(address + 2) != int(0xFFFF)) && (EEpromReadInt(address + 4) != int(0xFFFF))) {
            fullRotation[2] = EEpromReadInt(address);
            fullRotation[1] = EEpromReadInt(address + 2);
            fullRotation[0] = EEpromReadInt(address + 4);
            offset[2] = EEpromReadInt(address+6);
            offset[1] = EEpromReadInt(address+8);
            offset[0] = EEpromReadInt(address+10);
            secondsFlag = ((EEPROM.read(address+12) == 0) ? false : true);
            calibrationFlag = true;
            Serial.println(F("Aktuelle Kalibrierungswerte:"));
            
            Serial.print(F("Hour: \t"));
            Serial.print(fullRotation[2]);
            Serial.print(" \tOffset: ");
            Serial.println(offset[2]);

            Serial.print(F("Minute: \t"));
            Serial.print(fullRotation[1]);
            Serial.print(" \tOffset: ");
            Serial.println(offset[1]);
            
            Serial.print(F("Second: \t"));
            Serial.print(fullRotation[0]);
            Serial.print(" \tOffset: ");
            Serial.println(offset[0]);

        } else {
            calibrationFlag = false;
            Serial.println("Calibration necessary!");
        }
    // To interrupt the setup before everything starts      
    Serial.println(F("Druecke die weisse Taste oder <C> im Terminal zum Starten"));
    while (setButton.rose() == false){    // Auf Taste WEISS warten
        setButton.update();
        recvData();
        if ((newData == true) && (receivedChars[0] == 'c')) {  // oder auf Taste "c"
            newData = false;
            break;
        }    
    }
    setPressed = false;
    list_commands();

    // Run timer2 interrupt every 1 ms
    noInterrupts(); // disable all interrupts
    TCCR2A = 0;
    TCCR2B = 1 << CS22 | 0 << CS21 | 0 << CS20; // Timer Control Register:
    // Set Clock Select Bits means:
    // clkT2S/64 from prescaler, so the system clock is
    // divided by 64. If the clock frequence is 16 MHz the
    // interrupt is triggered every 1 ms.
    TIMSK2 |= 1 << TOIE2; // Timer2 Overflow Interrupt Enable
    // Timer 2 Interrupt Mask Register
    // TOIE2 set: Interrupt Timer 2 enabled
    interrupts(); // enable all interrupts

    onFlag = true;
    setupFlag = true;     //Setup-Modus, also keine Timeupdates 
    if (calibrationFlag == true){
        initialize();
    }
    else{
        calibration();    // Run once at first setup to fill the right values in EEProm
    }        
    // setupFlag = false;    //Timeupdates erlauben
}
SIGNAL(TIMER2_OVF_vect)
{
    interrupt_handling();
}
void interrupt_handling()
{
    // Button Minutes
    minButton.update();
    if (minButton.rose() == true) {
        minPressed = true;
    }
    // Button Hours
    hourButton.update();
    if (hourButton.rose() == true) {
        hourPressed = true;
    }
    // Button Setup
    setButton.update();
    if (setButton.rose() == true) {
        setPressed = true;
    }
    // Button Reserve
    resButton.update();
    if (resButton.rose() == true) {
        resPressed = true;
    }
    // Hall Sensors
    secHall.update();
    if (secHall.fell() == true) {
        hallDetect[0] = true;
    }
    minHall.update();
    if (minHall.fell() == true) {
        hallDetect[1] = true;
    }
    hourHall.update();
    if (hourHall.fell() == true) {
        hallDetect[2] = true;
    }
}
void loop()
{
    if (heartbeatFlag == true) {
        heartbeat(LEDBUILDINPIN);
    }
    // ***   Zehntelsekundentakt   *** (Abfrage alle 1/10 Sekunde)
    //     (Abfrage der Buttons)
    if ((millis() - Sekundenstart) >= Sekundentakt) {
        Sekundenstart = millis();
        checkButtons();
        checkHallSensors();
        if (setupFlag == false){        // Wenn die Setup-Routinen laufen kein Update der aktuellen Zeit
            time2Steps();
        }
        updateSteppers();
    };
    //***   Minutentakt   *** (eigentlich alle 10 Sekunden)
    if ((millis() - Minutenstart) >= Minutentakt) {
        Minutenstart = millis(); // Minutentakt neu starten
        printTime();
        checkInit();
    }
    recvData();
    computeData();
}
void recvData()
{
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    while (Serial.available() > 0) { //               && newData == false) {
        rc = Serial.read();
        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        } else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}
void computeData()
{
    // Command list:
    // A:           Toggle SetupFlag
    // B:           Inhibit Seconds
    // C:           Start Calibration
    // L:           List Commands
    // I:           Initialisierung (12:00 Uhr)
    // H<value>     Move Hour-Stepper    (0 for Zero-Point)
    // M<value>     Move Minutes-Stepper (0 for Zero-Point)
    // S<value>     Move Seconds-Stepper (0 for Zero-Point)
    // O:           Toggle OnFlag
    // P:           Print stored values
    // R:           Speichere Offsets
    // T:<00:00:00> Adjust the time
    // Z:           Start Setup Offsets

    steps = 0;
    if (newData == true) {
        switch (receivedChars[0]) {
        case 'a':           // SetupFlag ON / OFF
        case 'A':
            if(setupFlag == true){
                setupFlag = false;
                Serial.println(F("SetupFlag false"));    
            }
            else{
                setupFlag = true;
                Serial.println(F("SetupFlag true"));
            }
            break;
        case 'b':           // Inhibit seconds
        case 'B':
            if(secondsFlag == true){
                secondsFlag = false;
                Serial.println(F("Seconds OFF"));
                setupFlag = true;
                setZero(STEPPERSEC);
                setupFlag = false;    
            }
            else{
                secondsFlag = true;
                Serial.println(F("Seconds ON"));
            }
            EEpromWriteByte(address+12,(secondsFlag == false) ? 0 : 1);   
            break;
        case 'c':           // Calibration
        case 'C':
            calibration();
            break;
        case 'l':           // Print Command List
        case 'L':
            list_commands();
            break;    
        case 'i':           // Initialisierung to 12.00 Uhr
        case 'I':
            initialize();
            break;    
        case 'h':           // Hours Stepper
        case 'H':
            Serial.println(F("Hour-Stepper: "));
            if (receivedChars[1] == '+') {
                if (atoi(&receivedChars[2]) == 0) {
                    dirFlag[2] = true;
                    Serial.print(F("Set Zero forward: "));
                    Serial.print(setZero(STEPPERHOUR));
                    Serial.println(F(" Steps  OK"));
                    dirFlag[2] = true;
                } else {
                    steps = atoi(&receivedChars[2]);
                    toGoSteps[2] = steps;
                    Serial.print(F("Step-Counts Hours: "));
                    Serial.print(steps);
                }
            } else if (receivedChars[1] == '-') {
                if (atoi(&receivedChars[2]) == 0) {
                    dirFlag[2] = false;
                    Serial.print(F("Set Zero backward: "));
                    Serial.print(setZero(STEPPERHOUR));
                    Serial.println(F(" Steps  OK"));
                    dirFlag[2] = true;
                } else {
                    steps = -atoi(&receivedChars[2]);
                    toGoSteps[2] = steps;
                    Serial.print(F("Step-Counts Hours: "));
                    Serial.print(steps);
                }
            }
            // if (steps == 0) {
            //     Serial.print(F("Set Zero: "));
            //     Serial.print(setZero(STEPPERHOUR));
            //     Serial.println(F(" Steps  OK"));
            // } else {
            //     toGoSteps[2] = steps;
            //     Serial.print(F("Step-Counts Hours: "));
            //     Serial.print(steps);
            // }
            break;
        case 'm':           // Minutes Stepper
        case 'M':
            Serial.println(F("Minutes-Stepper: "));
           if (receivedChars[1] == '+') {
                if (atoi(&receivedChars[2]) == 0) {
                    dirFlag[1] = true;
                    Serial.print(F("Set Zero forward: "));
                    Serial.print(setZero(STEPPERMIN));
                    Serial.println(F(" Steps  OK"));
                    dirFlag[1] = true;
                } else {
                    steps = atoi(&receivedChars[2]);
                    toGoSteps[1] = steps;
                    Serial.print(F("Step-Counts Minutes: "));
                    Serial.print(steps);
                }
            } else if (receivedChars[1] == '-') {
                if (atoi(&receivedChars[2]) == 0) {
                    dirFlag[1] = false;
                    Serial.print(F("Set Zero backward: "));
                    Serial.print(setZero(STEPPERMIN));
                    Serial.println(F(" Steps  OK"));
                    dirFlag[1] = true;
                } else {
                    steps = -atoi(&receivedChars[2]);
                    toGoSteps[1] = steps;
                    Serial.print(F("Step-Counts Minutes: "));
                    Serial.print(steps);
                }
            }
            // steps = atoi(&receivedChars[2]);
            // if (steps == 0) {
            //     Serial.print(F("Set Zero: "));
            //     Serial.print(setZero(STEPPERMIN));
            //     Serial.println(F(" Steps  OK"));
            // } else {
            //     toGoSteps[1] = steps;
            //     Serial.print(F("Step-Counts Minutes: "));
            //     Serial.println(steps);
            // }
            break;
        case 's':           // Seconds Stepper
        case 'S':
            Serial.println(F("Seconds-Stepper: "));
            if (receivedChars[1] == '+') {
                if (atoi(&receivedChars[2]) == 0) {
                    dirFlag[0] = true;
                    Serial.print(F("Set Zero forward: "));
                    Serial.print(setZero(STEPPERSEC));
                    Serial.println(F(" Steps  OK"));
                    dirFlag[0] = true;
                } else {
                    steps = atoi(&receivedChars[2]);
                    toGoSteps[0] = steps;
                    Serial.print(F("Step-Counts Seconds: "));
                    Serial.print(steps);
                }
            } else if (receivedChars[1] == '-') {
                if (atoi(&receivedChars[2]) == 0) {
                    dirFlag[0] = false;
                    Serial.print(F("Set Zero backward: "));
                    Serial.print(setZero(STEPPERSEC));
                    Serial.println(F(" Steps  OK"));
                    dirFlag[0] = true;
                } else {
                    steps = -atoi(&receivedChars[2]);
                    toGoSteps[0] = steps;
                    Serial.print(F("Step-Counts Seconds: "));
                    Serial.print(steps);
                }
            }
            // steps = atoi(&receivedChars[2]);
            // if (steps == 0) {
            //     Serial.print(F("Set Zero: "));
            //     Serial.print(setZero(STEPPERSEC));
            //     Serial.println(F(" Steps  OK"));
            // } else {
            //     toGoSteps[0] = steps;
            //     Serial.print(F("Step-Counts Seconds: "));
            //     Serial.println(steps);
            // }
            break;
        case 'p':           // Print all Values
        case 'P':
            Serial.println();
            Serial.println(F("Aktuell gespeicherte Werte:"));
            Serial.println(F("==========================="));
            Serial.println(F("Seconds:"));
            Serial.print(F("  Offset:\t"));
            Serial.println(offset[0]);
            Serial.print(F("  Full Rotation:\t"));
            Serial.println(fullRotation[0]);
            Serial.print(F("  EEPROM-Value:\t"));
            Serial.println(EEpromReadInt(address+4));
            Serial.print(F("  Aktuelle Pos:\t"));
            Serial.println(aktPos[0]);

            Serial.println(F("Minutes:"));
            Serial.print(F("  Offset:\t"));
            Serial.println(offset[1]);
            Serial.print(F("  Full Rotation:\t"));
            Serial.println(fullRotation[1]);
            Serial.print(F("  EEPROM-Value:\t"));
            Serial.println(EEpromReadInt(address+2));
            Serial.print(F("  Aktuelle Pos:\t"));
            Serial.println(aktPos[1]);

            Serial.println(F("Hours:"));
            Serial.print(F("  Offset:\t"));
            Serial.println(offset[2]);
            Serial.print(F("  Full Rotation:\t"));
            Serial.println(fullRotation[2]);
            Serial.print(F("  EEPROM-Value:\t"));
            Serial.println(EEpromReadInt(address));
            Serial.print(F("  Aktuelle Pos:\t"));
            Serial.println(aktPos[2]);

            Serial.println();
            break;
        case 'r':           // Speichere aktuelle Offsets
        case 'R':
            Serial.println();
            Serial.println(F("Speichere die aktuellen Positionen in Offsets"));
            offset[0] = aktPos[0];
            offset[1] = aktPos[1];
            offset[2] = aktPos[2];
            aktPos[0] = 0;
            aktPos[1] = 0;
            aktPos[2] = 0;
            EEpromWriteInt(address+6, offset[2]);
            EEpromWriteInt(address+8, offset[1]);
            EEpromWriteInt(address+10, offset[0]);
                
            Serial.println(F("Seconds:"));
            Serial.print(F("  Offset:\t"));
            Serial.println(offset[0]);
            Serial.print(F("  Full Rotation:\t"));
            Serial.println(fullRotation[0]);
            Serial.print(F("  Aktuelle Pos:\t"));
            Serial.println(aktPos[0]);
            
            Serial.println(F("Minutes:"));
            Serial.print(F("  Offset:\t"));
            Serial.println(offset[1]);
            Serial.print(F("  Full Rotation:\t"));
            Serial.println(fullRotation[1]);
            Serial.print(F("  Aktuelle Pos:\t"));
            Serial.println(aktPos[1]);

            Serial.println(F("Hours:"));
            Serial.print(F("  Offset:\t"));
            Serial.println(offset[2]);
            Serial.print(F("  Full Rotation:\t"));
            Serial.println(fullRotation[2]);
            Serial.print(F("  Aktuelle Pos:\t"));
            Serial.println(aktPos[2]);

            Serial.println();
            break;
        case 'o':               // Toggle OnFlag (Moves / No Moves)
        case 'O':
            if (atoi(&receivedChars[1]) == 0) {
                onFlag = false;
                Serial.println(F("OFF"));
            } else {
                onFlag = true;
                Serial.println(F("ON"));
            };
            break;
        case 't':               // Set time via terminal
        case 'T':
            Serial.println("T:");
            parseTime();
            break;
        case 'z':           // Setup procedure to adjust the offsets
        case 'Z':
            setupFlag = true;
            Serial.println(F("Ich fahre die Stepper jetzt zu den Hall-Sensoren!"));
            Serial.println(F("..Anschliessend die Stepper mit <S><value>, <M><value>"));
            Serial.println(F("....und <H><value> auf 12 Uhr fahren."));
            Serial.println(F("......Dann die Offsets mit <R> speichern."));
            offset[0]=0;
            offset[1]=0;
            offset[2]=0;
            initialize();
            break;
        default:
            Serial.println(F("Was meintest Du?"));
            break;
        }
        newData = false;
    }
}
void list_commands(){
    // Print a list of available commands in console
    Serial.println();
    Serial.println(F("Command list:"));
    Serial.println(F("============="));
    Serial.println(F("A:\t\tToggle lSetupFlag"));
    Serial.println(F("B:\t\tInhibit Seconds (ON / OFF)"));
    Serial.println(F("C:\t\tStart Calibration"));
    Serial.println(F("L:\t\tPrint Command List"));
    Serial.println(F("I:\t\tInitialisierung"));
    Serial.println(F("H<+/-><value>\tMove Hour-Stepper (0 for Zero-Point)"));
    Serial.println(F("M<+/-><value>\tMove Minutes-Stepper (0 for Zero-Point)"));
    Serial.println(F("S<+/-><value>\tMove Seconds-Stepper (0 for Zero-Point)"));
    Serial.println(F("O:\t\tToggle OnFlag"));
    Serial.println(F("P:\t\tPrint aktuelle Werte"));
    Serial.println(F("R:\t\tSpeichere Offsets"));
    Serial.println(F("T:<00:00:00>\tSet the time"));
    Serial.println(F("Z:\t\tStart Setup Offsets"));
}
void moveSteps(uint8_t stepper, int steps)
{
    // Stepper fahren mit steps Schritten
    if(steps < 0){
        steps = int(round(float(steps)*BACKCORR)); 
    }
    if (onFlag == true) {
        switch (stepper) {
        case 0: // Seconds
            stepperSec.step(SEC_DIR * steps);
            break;
        case 1: // Minutes
            stepperMin.step(MIN_DIR * steps);
            break;
        case 2: // Hours
            stepperHour.step(HOUR_DIR * steps);
            break;
        }
    }
}
int setZero(uint8_t stepper){
    // Stepper fahren bis der Hallsensor anzeigt
    // Rückgabe: gefahrene Steps
    int counter_sec = 0;
    int counter_min = 0;
    int counter_hour = 0;
    switch (stepper) {
    case STEPPERSEC: // Seconds
        while (hallDetect[0] == false) {
            if (dirFlag[0] == true){
                moveSteps(0,increment[0]);
                counter_sec = counter_sec + increment[0];
            }
            else{
                moveSteps(0,-int(increment[0]));
                counter_sec = counter_sec - increment[0];
            }
            // Serial.print(F("Counter_SEC:\t"));
            // Serial.println(counter_sec);
        }
        hallDetect[0] = false;
        return counter_sec;
        break;
    case STEPPERMIN: // Minutes
        while (hallDetect[1] == false) { // dann drehen...
            if (dirFlag[1] == true){
                moveSteps(1,increment[1]);
                counter_min = counter_min + increment[1];
            }
            else{
                moveSteps(1,-int(increment[1]));
                counter_min = counter_min - increment[1];
            }
            // Serial.print(F("Counter_MIN:\t"));
            // Serial.println(counter_min);
        }
        hallDetect[1] = false;
        return counter_min;
        break;
    case STEPPERHOUR: //Hours
        while (hallDetect[2] == false) {
            if (dirFlag[2] == true){
                moveSteps(2,increment[2]);
                counter_hour = counter_hour + increment[2];
            }
            else{
                moveSteps(2,-int(increment[2]));
                counter_hour = counter_hour - increment[2];
            }
            // Serial.print(F("Counter_HOUR:\t"));
            // Serial.println(counter_hour);
        }
        hallDetect[2] = false;
        return counter_hour;
       break;
    default:
        return 0;
        break;   
    }
}
void printTime()
{
    DateTime now = rtc.now();   // RTC lesen
    stunde = now.hour();       // Stunden ablesen
    minute = now.minute();     // Minuten ablesen
    sekunde = now.second();    // Sekunden ablesen
    char s[20];
    Serial.print(F("Zeit: "));
    sprintf(s, "%02u:%02u:%02u", stunde, minute, sekunde);
    Serial.println(s);
}
void printDate()
{
    DateTime now = rtc.now();   // RTC lesen
    jahr = now.year();          // Stunden ablesen
    monat = now.month();        // Minuten ablesen
    tag = now.day();            // Sekunden ablesen
    char s[20];
    Serial.print(F("Datum: "));
    sprintf(s, "%02u.%02u.%04u", tag, monat, jahr);
    Serial.print(s);
    Serial.print("\t");
}
void heartbeat(int heartbeatPin)
{   // Heartbeat-LED zur Anzeige, ob der Prozessor noch lebendig ist.
    if ((millis() - blinkTime) > blinkDelay) {
        if (blinkFlag == true) {
            digitalWrite(heartbeatPin, LOW);
            blinkFlag = false;
            blinkDelay = long(blinkDelayOff);
        } else {
            digitalWrite(heartbeatPin, HIGH);
            blinkFlag = true;
            blinkDelay = long(blinkDelayOn);
        }
        blinkTime = millis();
    }
}
void checkButtons(){
    // Bedeutung der Buttons:
    // Rote Taste: Hours
    //      kurz drücken: Stunde um 1 vorstellen
    //      lang drücken: Stunde um 1 zurückstellen
    //                     
    // Schwarze Taste: Minutes
    //      kurz drücken: Minuten um 1 vorstellen
    //      lang drücken: Minuten um 1 zurückstellen
    //
    // Weisse Taste: Calibration
    //      1. kurz drücken: Initialisierung starten (Fahren auf 12:00 Uhr), auch um Uhr nach Stromausfall zu starten
    //                       danach setupFlag auf true stellen (keine Uhranzeige)
    //      2. mal kurz drücken: WiederAnzeige der Uhrzeit    
    //      lang drücken: Calibrierung starten, neue Fullrotation-Werte in EEPROM speichern
    //
    // Gelbe Taste: Sekunden ausschalten, Update ausschalten 
    //      kurz drücken: SecondsFlag toggeln:Sekundenzeiger auf 12:00 Uhr fahren und abschalten, Sekundenzeiger wieder anschalten
    //      lang drücken: OnFlag toggeln: Schaltet die Stepper aus.    
    
    if (hourPressed == true) {          //Button Hours  (Stellen der Uhrzeit: Stunden)   (ROTE TASTE)
        hourPressed = false;
        DateTime now = rtc.now();
        stunde = now.hour();
        minute = now.minute();
        if (hourButton.previousDuration() > 350) { // Länger als 350 msec gedrückt?
            Serial.println(F("HoursButton Long pressed!")); // Ja, dann LongPress
            dec2limits(stunde,0,23,stunde);
        }
        else{    
            Serial.println(F("Button Hours pressed"));
            inc2limits(stunde,0,23,stunde);
        }
        rtc.adjust(DateTime(2022,12,24,stunde,minute,0));
        displayZeit(stunde,minute);
    }
    if (minPressed == true) {           //Button Minutes (Stellen der Uhrzeit: Minuten)     (SCHWARZE TASTE)
        minPressed = false;
        DateTime now = rtc.now();
        stunde = now.hour();
        minute = now.minute();
        if (minButton.previousDuration() > 350) { // Länger als 350 msec gedrückt?
            Serial.println(F("MinButton Long pressed!")); // Ja, dann LongPress
            dec2limits(minute, 0, 59, minute);
        }
        else{
            Serial.println(F("Button Minutes pressed"));
            if (inc2limits(minute, 0, 59, minute) == true) {
                inc2limits(stunde, 0, 23, stunde);
            }
        }
        rtc.adjust(DateTime(2022,12,24,stunde,minute,0));
        displayZeit(stunde,minute);
    }
    if (setPressed == true) {       // Button Set (WEISSE TASTE)
        setPressed = false;
        if (setButton.previousDuration() > 350) { // Länger als 350 msec gedrückt?
            Serial.println(F("Button Calibration (Long) pressed!")); // Ja, dann LongPress
            // Calibrierung der Steps pro Runde starten. Ergebnis ins EEPROM speichern
            setupFlag = true;           // Dann in den Setup Mode
            calibration();              // Calibrieren
        }
        else{   //Initialisierung: Stepper auf 12:00 Uhr fahren und starten
            Serial.println(F("Button initialisation pressed!"));
            if (setupFlag == false){
                setupFlag = true;           // Dann in den Setup Mode
                initialize();               // Initialisieren    
            }
            else {
                setupFlag = false;          // Wieder in Anzeigemodus umschalten
            }
        }
    }
    // Button Seconds (Sekunden abschalten oder anschalten) (GELBE TASTE)
    if (resPressed == true){
        resPressed = false;
        if (resButton.previousDuration() > 350) { // Länger als 350 msec gedrückt?
            Serial.println(F("SecondsButton Long pressed!")); // Ja, dann LongPress
            // Tu etwas, wenn der Button lang gedrückt wurde
            // Toggle das OnFlag
            if (onFlag == true){            // OnFlag war ON
                onFlag = false;
            }
            else{                           // OnFlag war OFF            
                onFlag = true;
                setupFlag = true;           // Dann in den Setup Mode
                initialize();               // Initialisieren    
                setupFlag = false;          // und einschalten...
            }
        }    
        else { // Button short pressed
            Serial.println(F("Button Seconds pressed"));
            if (secondsFlag == true) {
                // Sekundenzeiger ausschalten
                secondsFlag = false;
                Serial.println(F("Sekundenzeiger ausgeschaltet"));
                setupFlag = true;
                setZero(STEPPERSEC);
                aktPos[0] = 0;
                lastSec = 0;
                setupFlag = false;
            } 
            else {
                // Sekundenzeiger anschalten
                secondsFlag = true;
                Serial.println(F("Sekundenzeiger eingeschaltet"));
            }
            EEpromWriteByte(address+12,(secondsFlag == false) ? 0 : 1);
        }
    }
}    
void displayZeit(byte hour, byte minutes)
{
    if (hour < 10)
        Serial.print(F(" "));
    Serial.print(hour, DEC);
    Serial.print(F(":"));
    if (minutes < 10)
        Serial.print(F("0"));
    Serial.print(minutes, DEC);
    Serial.print(F("   "));
    Serial.println();
}
void checkHallSensors()
{
    if (hallDetect[0] == true) {
        hallDetect[0] = false;
        Serial.println(F("Hall Sensor seconds detected"));
        // digitalWrite(LEDBUILDINPIN, HIGH);
        // delay(50);
        // digitalWrite(LEDBUILDINPIN, LOW);
    }
    if (hallDetect[1] == true) {
        hallDetect[1] = false;
        Serial.println(F("Hall Sensor minutes detected"));
        // digitalWrite(LEDBUILDINPIN, HIGH);
        // delay(50);
        // digitalWrite(LEDBUILDINPIN, LOW);
    }
    if (hallDetect[2] == true) {
        hallDetect[2] = false;
        Serial.println(F("Hall Sensor hours detected"));
        // digitalWrite(LEDBUILDINPIN, HIGH);
        // delay(50);
        // digitalWrite(LEDBUILDINPIN, LOW);
    }
}
void checkInit(){
    // Checks if it is time for initialization and starts it
    DateTime now = rtc.now();   // RTC lesen
    stunde = now.hour();       // Stunden ablesen
    minute = now.minute();     // Minuten ablesen
    sekunde = now.second();    // Sekunden ablesen
    if (stunde >= 12) {
        stunde = stunde - 12;
    }
    if ((stunde == INITHOUR) && (minute == INITMIN) && (sekunde < 30)){
        setupFlag = true;
        initialize();
        setupFlag = false;
    }
}
void binaryOutput(int var)
{
    unsigned int mask = 0x80; //binary:  1000 0000
    for (mask = 0x80; mask; mask >>= 1) {
        Serial.write(var & mask ? '1' : '0');
    }
    Serial.println();
}
void updateSteppers()
{
    // Überprüft für jeden Stepper, ob der Zielwert erreicht ist
    // Falls nein, werden die nächsten Schritte gemacht
    int steps;
    for (byte i = 0; i < 3; i++) { // Für jeden Stepper...
        if (toGoSteps[i] > 0) { // Sind noch Steps vorwärts zu machen?
            if (toGoSteps[i] > increment[i]) { // Sind noch mehr als das Increment zu machen
                steps = increment[i];
                toGoSteps[i] = toGoSteps[i] - increment[i]; // toGo anpassen
            } else { // Dann die restlichen Steps
                steps = toGoSteps[i];
                toGoSteps[i] = 0;
            }
            moveSteps(i, steps); // Und jetzt die Stepper fahren...
            aktPos[i] = aktPos[i] + steps;
            if (aktPos[i] > fullRotation[i]){   // Über die 12 hinaus....
                aktPos[i] = aktPos[i]-fullRotation[i];
            }
        }
        else if(toGoSteps[i] < 0) { // Sind noch Steps rückwärts zu machen?
            if (abs(toGoSteps[i]) > increment[i]) { // Sind noch mehr als das Increment zu machen
                steps = -increment[i];
                toGoSteps[i] = toGoSteps[i] + increment[i]; // toGo anpassen
            } else { // Dann die restlichen Steps
                steps = toGoSteps[i];
                toGoSteps[i] = 0;
            }
            moveSteps(i, steps); // Und jetzt die Stepper fahren...
            aktPos[i] = aktPos[i] + steps;
            if (aktPos[i] > fullRotation[i]){   // Über die 12 hinaus....
                aktPos[i] = aktPos[i] - fullRotation[i];
            }
            if (aktPos[i] < 0){
                aktPos[i] = fullRotation[i] + aktPos[i];
            }
        }
    }
}
void time2Steps()
{
    // Umsetzung der aktuellen Zeit in zu fahrende Steps
    DateTime now = rtc.now();       // RTC lesen
    stunde = now.hour();            // Stunden ablesen
    minute = now.minute();          // Minuten ablesen
    sekunde = now.second();         // Sekunden ablesen
    int zielHour=0;
    int zielMin=0;
    int zielSek=0;
    if (stunde >= 12) {
        stunde = stunde - 12;
    }
    if ((stunde != lastHour) || (int(minute / 5) != int(lastMin / 5))) {  //Zwischenraum zwischen jeder Stunde in 5 Teile teilen
        // Steps für Stunden berechnen
        zielHour = int(round((float(fullRotation[2]) / 60) * ((float(stunde) * 5) + float(minute) / 12)));
        // Serial.print("FullRotation / 60:\t");
        // Serial.print(fullRotation[2]/60);
        // Serial.print("\tFloat FR:\t");
        // Serial.println(float(fullRotation[2]) / 60);
        
        // Serial.print("Stunde * 5:\t\t");
        // Serial.print(stunde * 5);
        // Serial.print("\tFloat:  ");
        // Serial.println(float(stunde) * 5);
       
        // Serial.print("Minute /12:\t\t");
        // Serial.print(minute / 12);
        // Serial.print("\tFloat:\t");
        // Serial.print(float(minute) / 12);
        // Serial.print("\tINT:\t");
        // Serial.println(int(float(minute) / 12));

        // Serial.print("2.Ausdruck komplett:\t");
        // Serial.println((float(stunde)*5)+float(minute)/12);
                
        // Serial.print("Zielhour:\t");
        // Serial.print(zielHour);
        // Serial.print("   Float:\t");
        // Serial.println((float(fullRotation[2]) / 60) * ((float(stunde) * 5) + float(minute) / 12));

        // Vorwärts und rückwärts
        if (abs(zielHour - aktPos[2]) < fullRotation[2] - abs(zielHour - aktPos[2])) {
            toGoSteps[2] = zielHour - aktPos[2];
        } else {
            if (aktPos[2] > zielHour) {
                toGoSteps[2] = fullRotation[2] + zielHour - aktPos[2];
            } else {
                toGoSteps[2] = -fullRotation[2] + zielHour - aktPos[2];
            }
        }
        // Nur vorwärts
        // if((zielHour - aktPos[2]) < 0){
        //     toGoSteps[2] = fullRotation[2] + (zielHour - aktPos[2]);
        // }
        // else {
        //     toGoSteps[2] = zielHour - aktPos[2];
        // }

        lastHour = stunde;
        Serial.print(F(" fullRotation[2] = "));
        Serial.print(fullRotation[2]);
        Serial.print(F("\t aktPos[2] = "));
        Serial.print(aktPos[2]);
        Serial.print(F("\t toGo[2] = "));
        Serial.print(toGoSteps[2]);
        Serial.print(F("\t lastHour = "));
        Serial.print(lastHour);
        Serial.print(F("\t aktHour = "));
        Serial.println(stunde);
    }
    if (minute != lastMin) {
        // Steps für Minuten berechnen
        zielMin = int(round((float(fullRotation[1]) / 60) * minute));
        // Vorwärts und rückwärts
        if (abs(zielMin - aktPos[1]) < fullRotation[1] - abs(zielMin - aktPos[1])) {
            toGoSteps[1] = zielMin - aktPos[1];
        } else {
            if (aktPos[1] > zielMin) {
                toGoSteps[1] = fullRotation[1] + zielMin - aktPos[1];
            } else {
                toGoSteps[1] = -fullRotation[1] + zielMin - aktPos[1];
            }
        }
        //    Nur vorwärts:
        // if ((zielMin - aktPos[1])< 0){
        //     toGoSteps[1] = fullRotation[1]+(zielMin - aktPos[1]);
        // } 
        // else {
        //     toGoSteps[1] = zielMin - aktPos[1];
        // }

        lastMin = minute;
        Serial.print(F(" fullRotation[1] = "));
        Serial.print(fullRotation[1]);
        Serial.print(F("\t aktPos[1] = "));
        Serial.print(aktPos[1]);
        Serial.print(F("\t toGo[1] = "));
        Serial.print(toGoSteps[1]);
        Serial.print(F("\t lastMin = "));
        Serial.print(lastMin);
        Serial.print(F("\t aktMin = "));
        Serial.println(minute);
    }
    if (secondsFlag == true) {
        if (sekunde != lastSec) {
            // Steps für Sekunden berechnen
            zielSek = int(round((fullRotation[0] / 60) * sekunde));
            // Vorwärts und rückwärts
            if (abs(zielSek - aktPos[0]) < fullRotation[0] - abs(zielSek - aktPos[0])) {
                toGoSteps[0] = zielSek - aktPos[0];
            } else {
                if (aktPos[0] > zielSek) {
                    toGoSteps[0] = fullRotation[0] + zielSek - aktPos[0];
                } else {
                    toGoSteps[0] = -fullRotation[0] + zielSek - aktPos[0];
                }
            }
            // Nur vorwärts
            // if ((zielSek - aktPos[0]) < 0 ){
            //     toGoSteps[0] = fullRotation[0] +(zielSek -aktPos[0]);           
            // }
            // else{
            //     toGoSteps[0] = zielSek - aktPos[0];    
            // }
            lastSec = sekunde;
        }
    }
}
void initialize(){
    // Initialisierungsroutinen beim Einschalten:
    // - zum Sensor fahren
    // - Offset-Werte in aktPos speichern.
    // FALLS Initialisierung zulange dauert, dann stoppen und offline schalten (Sicherungsmaßnahme)

    // Zunächst zum Sensor fahren
    Serial.println(F("Zum Start zum Sensor fahren ..."));
    Serial.println(F("...Stundenrad..."));
    setZero(STEPPERHOUR);
    moveSteps(2, offset[2]);
    aktPos[2] = 0;
    lastHour = 0;
    
    Serial.println(F("...Minutenrad..."));
    setZero(STEPPERMIN);
    moveSteps(1, offset[1]);
    aktPos[1] = 0;
    lastMin = 0;
    if (secondsFlag == true){
        Serial.println(F("...Sekundenrad..."));
        setZero(STEPPERSEC);
        moveSteps(0, offset[0]);
        aktPos[0] = 0;
        lastSec = 0;
    }       
    Serial.println(F("Alle Zeiger auf 12.00 Uhr!"));
    Serial.println();
}

void calibration(){
    // Für Sec, Min, Hour: Kugel einmal rund bewegen bis Hall-Sensor anspricht,
    // Position auf 0 stellen, dann weiterdrehen, bis Hall-Sensor nicht mehr 
    // anspricht. Dann Weiterdrehen, bis der Hall-Sensor wieder anspricht. Position merken.
    // Daraus ergibt sich die Anzahl Steps für eine volle Umdrehung.
    // 
    // - zum Sensor fahren
    // - eine Runde fahren zur Kalibrierung
    // - Dabei die benötigten Steps merken
    // - Werte aus EEPROM lesen und mit den aktuellen Werten vergleichen: 
    //      Falls Abweichung zu groß-> verwerfen
    //      Falls Abweichung 0 oder in Toleranz: Mittelwert bilden und neue Werte Speichern
    // - Neue Werte in EEPROM speichern
    // FALLS Initialisierung zulange dauert, dann stoppen und offline schalten (Sicherungsmaßnahme)
    calstarttime = millis();

    setupFlag = true;
    onFlag = true;
    // Zunächst zum Sensor fahren
    Serial.println(F("Fahre zum Sensor ..."));
    Serial.println(F("...Stundenrad..."));
    setZero(STEPPERHOUR);
    Serial.println(F("...Minutenrad..."));
    setZero(STEPPERMIN);
    Serial.println(F("...Sekundenrad..."));
    setZero(STEPPERSEC);
    // Dann Kalibrierung starten
    Serial.println(F("Starting Calibration..."));
    // moveSteps(STEPPERHOUR,400);
    hourSteps = 0;
    hourSteps = setZero(STEPPERHOUR);
    Serial.print(F("Full rotation HourStepper:\t"));
    Serial.println(hourSteps);
    // moveSteps(STEPPERMIN,400);
    minuteSteps = 0;
    minuteSteps = setZero(STEPPERMIN);
    Serial.print(F("Full rotation MinuteStepper:\t"));
    Serial.println(minuteSteps);
    // moveSteps(STEPPERSEC,400);
    secondSteps = 0;
    secondSteps = setZero(STEPPERSEC);
    Serial.print(F("Full rotation SecondStepper:\t"));
    Serial.println(secondSteps);

    // Werte übernehmen
    fullRotation[2] = hourSteps;
    fullRotation[1] = minuteSteps;
    fullRotation[0] = secondSteps;
    calibrationFlag = true; 

    // Werte im EEPROM speichern
    Serial.println(F("Writing Values to EEPROM..."));
    Serial.println();
    EEpromWriteInt(address, hourSteps);
    EEpromWriteInt(address+2, minuteSteps);
    EEpromWriteInt(address+4, secondSteps);

    // Auf 12 Uhr stellen:
    // moveSteps(0, offset[0]);
    // aktPos[0] = 0;
    // moveSteps(1, offset[1]);
    // aktPos[1] = 0;
    // moveSteps(2, offset[2]);
    // aktPos[2] = 0;
//     setupFlag = false;
}

void parseTime()
{
    // Read the time from user-input (console) and adjust the RTC
    // Use 00:00:00 as format    
    char Console1[40];
    int Hour =      getIntFromString(receivedChars,1);
    int Minute =    getIntFromString(receivedChars,2);
    int Second =    getIntFromString(receivedChars,3);
    // check if the values are valid
    boolean validTime = (inRange(Hour, 0, 23) && inRange(Minute, 0, 59) && inRange(Second, 0, 59));
    if (validTime == true){
       sprintf(Console1 , "Time set to %02d:%02d:%02d", Hour, Minute, Second);
       Serial.println(Console1);
       rtc.adjust(DateTime(2022,12,05,Hour,Minute,Second));
    }
}
boolean inRange (int value, int minvalue, int maxvalue){
    //return (value >= minvalue && value <= maxvalue)? true : false;
    if (value >= minvalue && value <=maxvalue){
        return true;
    }
    else{ 
        return false;
    }
}
void EEpromWriteByte(int address, byte value)
{
    byte akt_value;
    akt_value = EEPROM.read(address);
    if (akt_value != value) {
        EEPROM.write(address, value);
    }
}
void EEpromWriteInt(int address, int value)
{
    // Schreibt Integer value in EEPROM-Adresse address falls notwendig    
    byte lowByte = ((value >> 0) & 0xFF);
    byte highByte = ((value >> 8) & 0xFF);
    int akt_value = EEpromReadInt(address);
    // Write only if value is different to prevent unneccessary write cycles
    if (value != akt_value) {
        EEPROM.write(address, lowByte);
        EEPROM.write(address + 1, highByte);
    }
}
int EEpromReadInt(int address)
{   // Liesst 2-Byte Integer aus EEPROM-Adresse address und gibt den Wert zurück
    byte lowByte = EEPROM.read(address);
    byte highByte = EEPROM.read(address + 1);
    return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}
boolean inc2limits(byte zahl, byte minimum, byte maximum, byte& ret){
    ret = zahl + 1;
    if (ret > maximum) {
        ret = minimum;
        return true;
    } else {
        return false;
    }
}
boolean dec2limits(byte zahl, byte minimum, byte maximum, byte& ret){
    ret = zahl - 1;
    if (ret == 255) {
        ret = maximum;
        return true;
    }
    if (ret < minimum) {
        ret = minimum;
        return true;
    } else {
        return false;
    }
}

int getIntFromString (char *stringWithInt, byte num)
// Extrahiert Integervalue aus string.
// AUFGEPASST: Größe des char-Arrays!
//
// Taken from User Jurs from Arduino Forum: https://forum.arduino.cc/t/int-aus-string-extrahieren/143685
// input: pointer to a char array
// returns an integer number from the string (positive numbers only!)
// num=1, returns 1st number from the string
// num=2, returns 2nd number from the string, and so on
{
  char *tail; 
  while (num>0)
  {
    num--;
    // skip non-digits
    while ((!isdigit (*stringWithInt))&&(*stringWithInt!=0)) stringWithInt++;
    tail=stringWithInt;
    // find digits
    while ((isdigit(*tail))&&(*tail!=0)) tail++;
    if (num>0) stringWithInt=tail; // new search string is the string after that number
  }  
  return(strtol(stringWithInt, &tail, 0));
}
