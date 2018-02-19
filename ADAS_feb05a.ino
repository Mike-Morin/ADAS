#include "BLESerial.h"

#include "MS5607/IntersemaBaro.h"

#include <CurieTimerOne.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <CurieIMU.h>
//#include <CurieBLE.h>

#define ADAS_ERROR 2 //Allowed error in ADAS motion due to overshoot. (encoder pulses) 
#define ADAS_SLOW_THRESH 10 // Number of pulses away from target at which ADAS slows down. (encoder pulses)
#define ADAS_MAX_JERK_TIME 500 // Amount of time ADAS slowing is to be applied before jerk condition is cleared. (milliseconds)
#define ADAS_MAX_DEPLOY 220 // Max number of ADAS pulses--this is where the fins disengage.(encoder pulses)
#define ADAS_PWM_FREQ 75 // PWM frequency for slow condition. (#/255)
#define LAUNCH_THRESHOLD_TIME 200 //(ms)
#define LAUNCH_THRESHOLD_ACC 4 //(g)

#define WDUS 1000000 //Number of microseconds until watchdog times out and e-stops ADAS. (1000000us = 1s)

/* Pin defs */
const byte hbridgeIN1pin = 2; //h-bridge board pins 2 & 3
const byte beeperpin = 3;
const byte limitswitchpin = 4;
const byte hbridgeENpin = 5;
const byte hbridgeIN2pin = 6;//h-bridge board pins  1 & 4
const byte encoderpinA = 7;
const int  sdpin = 10; // Also known as SS.


/* Pin defs in the SPI library
   SS	-- pin 10
   MOSI -- pin 11
   MISO -- pin 12
   SCK  -- pin 13
*/



/* ADAS variables */
typedef struct {
  boolean launched = false;            //Set to true when accelerometer detects launch condition.
  volatile int dir = 0;                //Current ADAS direction.
  int lastdir = -3;                     //Previous ADAS direction.
  volatile int pulsect = 0;            //Current pulse count. Set by interrupt in ADASpulse();
  int desiredpos = 0;                 // Set this and run ADASupdate() to move ADAS to that position. (pulses)
  volatile boolean atlimit = false;    //True when limit switch is hit. Updated by ADASupdate() and ADAShitlimit().
  unsigned int jerktime = 0; // Time change since last state change. (ms)
  boolean slow = false;               // ADAS is in slow mode.
  int error = -99;
  boolean emergencystop = false;
  boolean jerk = true;
} ADASstate;

ADASstate ADAS;

/* Data variables */
float ADASdatabuf[16][10];

Intersema::BaroPressure_MS5607B MS5607alt(true);

File ADASdatafile;


void ADASWDtimeout() {
  /* Executed by main watchdog timer if it times out */

  ADAS.emergencystop = true; // emergency stop
  digitalWrite(hbridgeIN1pin, HIGH); // stop the motor completely
  digitalWrite(hbridgeIN2pin, HIGH);
  ADAS.desiredpos = ADAS.pulsect;
  ADASbeep(-99); // beep
}

void ADASbeep(int code) {
  /*
    Plays informational beeps, codes are integers, negative codes are errors,
    positive codes are purely informational
    error code -99 is an extreme error and is indicated by a constant tone
  */

  if (code == -99) { //Motor emergency stop. Beep forever.
    digitalWrite(beeperpin, HIGH);
    return;
  }

  if (code < 0) {
    // Critical Errors are negative
    // if there is a critical error there is a 2 second long beep before the code
    digitalWrite(beeperpin, HIGH);
    delay(2000);
  } else {
    // any non negative code is a information code
    // there is a 400ms long beep before these codes
    digitalWrite(beeperpin, HIGH);
    delay(400);
  }
  digitalWrite(beeperpin, LOW);
  delay(1000);
  // beep out the code
  for (int i = 0; i < abs(code); i++) {
    digitalWrite(beeperpin, HIGH);
    delay(200);
    digitalWrite(beeperpin, LOW);
    delay(200);
  }
}

void ADASpulse() {
  /*
    Whenever a pulse is detected, an interrupt is triggered and this function executes.
    The direction is set by the ADASupdate() function. Using both encoders would allow
    us to measure ADAS direction directly, but the pulses from both encoders must be
    compared. This is so slow that it was introducing weirdness. We could try again
    using port manipulation but the indirect approach here seems reliable.
  */


  if (ADAS.emergencystop) { //This is a fatal condition. ADAS must be reset to clear.
    ADAS.desiredpos = ADAS.pulsect;
    ADASbeep(-99);
    return;
  }
  if (ADAS.dir == 1) {
    ADAS.pulsect++;
  } else if (ADAS.dir == -1)  {
    ADAS.pulsect--;
  }

  if ((ADAS.pulsect >= (ADAS.desiredpos - ADAS_ERROR)) && (ADAS.pulsect <= (ADAS.desiredpos + ADAS_ERROR))) {
    ADAS.dir = 0;
    digitalWrite(hbridgeIN1pin, HIGH);
    digitalWrite(hbridgeIN2pin, HIGH);
    digitalWrite(hbridgeENpin, LOW);

  }
  if (ADAS.pulsect >= ADAS_MAX_DEPLOY) {
    ADAS.dir = 0;
    ADAS.desiredpos = ADAS_MAX_DEPLOY;
    digitalWrite(hbridgeIN1pin, HIGH);
    digitalWrite(hbridgeIN2pin, HIGH);
    digitalWrite(hbridgeENpin, LOW);
  }
}

void ADASclose() {
  /* Closes ADAS all the way. Use for retracting fins after apogee.*/

  if (!digitalRead(limitswitchpin)) { //ADAS is open
    ADAS.desiredpos = 0 - ADAS_MAX_DEPLOY; //Close ADAS for sure. Relies on the limit switch working.
  }
}

void ADASzero() {
  /*
    Stops the motor when the limit switch is engaged
    called on interupt from optical limit switch
  */
  Serial.println("Limit Switch Triggered");
  ADAS.pulsect = 0;
  digitalWrite(hbridgeIN1pin, HIGH);
  digitalWrite(hbridgeIN2pin, HIGH);
}


void ADASmove() {
  /*
    Actually moves the ADAS motor according to ADAS.desiredpos and ADAS.pulsect.
  */
  static unsigned int lastmillis = 0;
  if (ADAS.emergencystop) {
    digitalWrite(hbridgeIN1pin, HIGH);
    digitalWrite(hbridgeIN2pin, HIGH);
    ADASbeep(-99);
    return;
  }
  if (ADAS.slow) {
    analogWrite(hbridgeENpin, ADAS_PWM_FREQ);
  } else {
    digitalWrite(hbridgeENpin, HIGH);
  }
  if (ADAS.dir == 1) { //forward
    digitalWrite(hbridgeIN1pin, HIGH);
    digitalWrite(hbridgeIN2pin, LOW);
  } else if (ADAS.dir == -1) { //reverse
    digitalWrite(hbridgeIN1pin, LOW);
    digitalWrite(hbridgeIN2pin, HIGH);
  } else if (ADAS.dir == 0) { // STOP
    ADAS.slow = false;
    digitalWrite(hbridgeIN1pin, HIGH);
    digitalWrite(hbridgeIN2pin, HIGH);
  }
}


void ADASupdate() {
  /*
    Moves ADAS according to ADAS.desiredpos. Incorporates hysteresis via ADAS_ERROR
    so the motor doesn't overshoot its target position and go into violent
    oscillations.
  */

  static unsigned int lastjerk = 0;

  ADASmove(); //ADAS won't move unless ADASmove() is called here.

  if (ADAS.lastdir != ADAS.dir) { //Prevents sudden changes by setting slow flag.
    ADAS.jerk = true;
    ADAS.slow = true;
    lastjerk =  millis();
  }

  if ((millis() - lastjerk) >= ADAS_MAX_JERK_TIME && ADAS.jerk == true) {
    ADAS.slow = false;
    ADAS.jerk = false;
  }

  if (ADAS.pulsect <= (ADAS.desiredpos - ADAS_ERROR)) { // Need to go forward to achieve target pos.
    ADAS.dir = 1;
    ADAS.lastdir = ADAS.dir;
    if (ADAS.pulsect >= (ADAS.desiredpos - ADAS_SLOW_THRESH)) { //slow when approaching target
      ADAS.slow = true;
    }
  } else if (ADAS.pulsect >= (ADAS.desiredpos + ADAS_ERROR)) { // Need to go reverse to achieve target pos.
    ADAS.dir = -1;
    ADAS.lastdir = ADAS.dir;
    if ( ADAS.pulsect <= (ADAS.desiredpos + ADAS_SLOW_THRESH)) { //slow when approaching target
      ADAS.slow = true;
    }
  }
}


void isLaunch() {
  /*
    Launch detection code. If the total acceleration on the system
    is above LAUNCH_THRESHOLD_ACC for a continuous LAUNCH_THRESHOLD_TIME,
    the ADAS.launched flag is set.
  */

  boolean nolaunch = false;
  static unsigned int lastmillis = 0;
  if (millis() - lastmillis > LAUNCH_THRESHOLD_TIME) {
    lastmillis = millis();
    for (int i = 0; i < 10 && !nolaunch; i++) {
      if ((sqrt(pow(ADASdatabuf[1][i], 2) + pow(ADASdatabuf[2][i], 2) + pow(ADASdatabuf[3][i], 2))) < LAUNCH_THRESHOLD_ACC) {
        Serial.println(sqrt(pow(ADASdatabuf[1][i], 2) + pow(ADASdatabuf[2][i], 2) + pow(ADASdatabuf[3][i], 2)));
        nolaunch = true;
      }
    }

    if (!nolaunch) {
      ADAS.launched = true;
    }
  }
}


void getData() {
  /*
    Polls all the sensors and puts the data in the ADASdatabuf.
    The acclerometer gets polled 10 times for every altimeter
    reading because the altimeter is very slow (fix!?).
  */

  for (int i = 0; i < 10; i++) { //The acclerometer is polled 10 times.
    ADASdatabuf[0][i] = micros();

    CurieIMU.readAccelerometerScaled(ADASdatabuf[1][i], ADASdatabuf[2][i], ADASdatabuf[3][i]);
    CurieIMU.readGyroScaled(ADASdatabuf[4][i], ADASdatabuf[5][i], ADASdatabuf[6][i]);
    ADASdatabuf[7][i] = (CurieIMU.readTemperature() / 512.0 + 23);

    if (i == 0) { // The altimeter is polled once.
      ADASdatabuf[8][i] = MS5607alt.getHeightCentiMeters();
    } else {
      ADASdatabuf[8][i] = ADASdatabuf[8][i - 1];
    }
  }
}


void writeData() {
  /*
    Writes all the sensor data to SD card
  */

  // TODO: this could probably be simplified and/or be made faster by preformatting the string that is pushed to the sd card
  ADASdatafile = SD.open("ADASdata.txt", FILE_WRITE);

  if (ADASdatafile) {
    for (int j = 0; j < 10; j++) { //10 blocks of
      for (int i = 0; i < 9; i++) { //9 sensor outputs
        ADASdatafile.print(ADASdatabuf[i][j]);
        ADASdatafile.print("\t");
      }
      ADASdatafile.print("\t");
      ADASdatafile.print(ADAS.launched);
      ADASdatafile.print("\t");
      ADASdatafile.print(ADAS.pulsect);
      ADASdatafile.print("\t");
      ADASdatafile.print(ADAS.desiredpos);
      ADASdatafile.print("\t");
      ADASdatafile.print(ADAS.dir);
      ADASdatafile.print("\n");
    }
    ADASdatafile.close();
  } else {
    // if the file didn't open, print an error:
    // NOTE: this doesnt do much in the air
    Serial.println("error opening test.txt");
    ADAS.error = -9;
    ADASbeep(-9);
  }
}


void ADASlaunchtest() {
  /*
    Test launch code for model testing
  */

  static unsigned int curmillis = 0;
  // if (ADAS.launched) {
  if (curmillis == 0) {
    curmillis = millis();
  }
  if (millis() - curmillis >= 1000 && millis() - curmillis < 2000) {
    ADAS.desiredpos = 0;
  }
  if (millis() - curmillis >= 2000 && millis() - curmillis < 3000) {
    ADAS.desiredpos = 50;
  }
  if (millis() - curmillis >= 3000 && millis() - curmillis < 4000) {
    ADAS.desiredpos = 75;
  }
  if (millis() - curmillis >= 5000 && millis() - curmillis < 6000) {
    ADAS.desiredpos = 100;
  }
  if (millis() - curmillis >= 7000 && millis() - curmillis < 8000) {
    ADAS.desiredpos = 200;
    curmillis = millis(); //remove this for actual launches! causes repeat.

  }
  if (millis() - curmillis >= 60000) {
    //ADASclose();
  }
  //}
}


int ADASselftest() {
  /*
    Checks the fins for proper behavior and
    verifies that h-bridge and motor
    connections are correct. Also does an
    SD card write and read test. Returns 0 if
    everything is OK. A negative if not so.
  */
  static int lastpulsect = ADAS.pulsect;

  for ( int i = 0; i < 1000 && lastpulsect == ADAS.pulsect; i++) {
    digitalWrite(hbridgeENpin, HIGH);
    digitalWrite(hbridgeIN1pin, HIGH); //test stop condition 1
    digitalWrite(hbridgeIN2pin, HIGH);
    digitalWrite(hbridgeIN1pin, LOW); //test stop condition 2
    digitalWrite(hbridgeIN2pin, LOW);
    delay(10);
  }
  if (lastpulsect != ADAS.pulsect) { // Stop condition falure.
    ADAS.error = -2;
    return ADAS.error;
  }
  ADAS.desiredpos = ADAS.pulsect + ADAS_ERROR + 2; //test forward condition.
  for (int i = 0; i < 100; i++) {
    ADASupdate();
    delay(1);
  }
  if (ADAS.pulsect < lastpulsect) { //Polarity swapped.
    ADAS.error = -3;
    ADAS.emergencystop = true;
    return ADAS.error;
  }
  if (ADAS.pulsect == lastpulsect) { //Motor not moving.
    ADAS.error = -4;
    ADAS.desiredpos = 0;
    return ADAS.error;
  }

  ADAS.desiredpos = ADAS.pulsect - ADAS_ERROR - 2; //test reverse condition.
  for (int i = 0; i < 1000; i++) {
    ADASupdate();
    delay(1);
  }
  if (ADAS.pulsect > lastpulsect) { //Polarity swapped.
    ADAS.error = -5;
    ADAS.desiredpos = 0;
    return ADAS.error;
  }
  if (ADAS.pulsect == lastpulsect) { //Motor not moving this time. Reverse is not working.
    ADAS.error = -6;
    ADAS.desiredpos = 0;
    return ADAS.error;
  }

  ADAS.desiredpos = ADAS_MAX_DEPLOY; //test full range
  for (int i = 0; i < 2000 && lastpulsect == ADAS.pulsect; i++) {
    ADASupdate();
    delay(10);
  }
  ADAS.desiredpos = 50;
  for (int i = 0; i < 2000 && lastpulsect == ADAS.pulsect; i++) {
    ADASupdate();
    delay(10);
  }
  if ((ADAS.pulsect > (50 + ADAS_ERROR)) || (ADAS.pulsect < (50 - ADAS_ERROR))) { //Motor indexing issue.
    ADAS.error = -7;
    return ADAS.error;
  }

  ADAS.desiredpos = 0 - ADAS_ERROR - 1;
  for (int i = 0; i < 1000 && lastpulsect == ADAS.pulsect; i++) {
    ADASupdate();
    delay(10);
  }
  if ((ADAS.pulsect > (0 + ADAS_ERROR)) || (ADAS.pulsect < (0 - ADAS_ERROR))) { //Motor past limit. Fins disengaged or limit switch failure.
    ADAS.error = -8;
    ADAS.emergencystop = true;
    return ADAS.error;
  }

  char ADASteststring[11] = "TESTING123";
  File ADAStestfile;
  ADAStestfile = SD.open("ADAStestfile.txt", FILE_WRITE);
  ADAStestfile.println(ADASteststring);
  ADASdatafile.close();

  ADAStestfile = SD.open("ADAStestfile.txt");
  char readtestbuf[11];
  for (int i = 0; i < 11 && ADASdatafile.available(); i++) {
    if (ADAStestfile.read() != ADASteststring[i]) {
      ADAS.error = -9;  //File read/write error.
      return ADAS.error;
    }
  }
  ADAStestfile.close();
  ADAS.error = 0;
  return ADAS.error; // All tests passed.
}

void setup() {
  BLESerial.setName("ADAS");
  BLESerial.begin();

  Serial.begin(9600);

  /* For the altimeter */
  MS5607alt.init();

  /* For the built-in Curie IMU */
  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(16);
  CurieIMU.setGyroRate(3200);
  CurieIMU.setAccelerometerRate(1600);

  while (!SD.begin(sdpin)) { //Stop everything if we cant see the SD card!
    Serial.println("Card failed or not present.");
    ADASbeep(-1);
    delay(1000);
  }

  Serial.println("Card OK");
  ADASbeep(1);

  delay(500);


  /* ADAS control stuff */
  pinMode(hbridgeIN1pin, OUTPUT); //hbridge IN1
  pinMode(hbridgeIN2pin, OUTPUT); //hbridge IN2
  pinMode(hbridgeENpin, OUTPUT); //hbridge EN pin for pwm
  pinMode(encoderpinA, INPUT); //encoder A (or B... either works).
  pinMode(limitswitchpin, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderpinA), ADASpulse, RISING); //Catch interrupts from the encoder.
  attachInterrupt(digitalPinToInterrupt(limitswitchpin), ADASzero, FALLING); // catch when the limit switch is disengaged


  /* Run self-test until pass. */
  while (ADAS.error != 0) {
    ADASbeep(ADASselftest());
    }

  CurieTimerOne.start(WDUS, &ADASWDtimeout); //Starts watchdog timer.
}


void loop() {
  CurieTimerOne.restart(WDUS); //Restarts watchdog timer.
  BLESerial.println("Hello, this is ADAS.");
  getData();
  writeData();
  if (!ADAS.launched) { // why check if the rocket has lauched after it has launched?
    isLaunch();
  }
  ADASupdate();
  ADASlaunchtest();
  Serial.println(ADAS.launched);
  Serial.println(ADAS.error);


}

