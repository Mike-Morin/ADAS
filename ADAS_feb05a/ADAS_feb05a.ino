  //#include "BLESerial/BLESerial.h"

#include "MS5607/IntersemaBaro.h"

#include <CurieTimerOne.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <CurieIMU.h>
#include "MPU6050.h"
#include "MadgwickAHRS.h"

#define ADAS_ERROR 2 //Allowed error in ADAS motion due to overshoot. (encoder pulses) 
#define ADAS_SLOW_THRESH 10 // Number of pulses away from target at which ADAS slows down. (encoder pulses)
#define ADAS_MAX_JERK_TIME 500 // Amount of time ADAS slowing is to be applied before jerk condition is cleared. (milliseconds)
#define ADAS_MAX_DEPLOY 220 // Max number of ADAS pulses--this is where the fins disengage.(encoder pulses)
#define ADAS_PWM_FREQ 255 // PWM frequency for slow condition. (#/255)
#define LAUNCH_THRESHOLD_TIME 200 //(ms)
#define LAUNCH_THRESHOLD_ACC 4 //(g)

#define WDUS 2000000 //Number of microseconds until watchdog times out and e-stops ADAS. (1000000us = 1s)

/* Pin defs */
const byte hbridgeIN1pin = 2; //h-bridge board pins 2 & 3
const byte beeperpin = 3;
const byte limitswitchpin = 4;
const byte hbridgeENpin = 5;
const byte hbridgeIN2pin = 6;//h-bridge board pins  1 & 4
const byte encoderpinA = 7;
const int  sdpin = 10; // Also known as SS.


/* Pin defs in the SPI library
   SS -- pin 10
   MOSI -- pin 11
   MISO -- pin 12
   SCK  -- pin 13
*/



/* ADAS variables */
typedef struct {
  boolean launched = false;            //Set to true when accelerometer detects launch condition.
  unsigned long launch_time = 0;
  volatile int dir = 0;                //Current ADAS direction.
  int lastdir = -3;                     //Previous ADAS direction.
  volatile int pulse_count = 0;            //Current pulse count. Set by interrupt in ADASpulse();
  int desiredpos = 0;                // Set this and run ADASupdate() to move ADAS to that position. (pulses)
  volatile boolean atLimit = false;    //True when limit switch is hit. Updated by ADASupdate() and ADAShitlimit().
  unsigned int jerktime = 0; // Time change since last state change. (ms)
  boolean slow = false;               // ADAS is in slow mode.
  int error = -99;
  boolean inFatalError = false;
  boolean jerk = true;
} ADASstate;

ADASstate ADAS;
/* Data variables */
float ADASdatabuf[18][10];

Intersema::BaroPressure_MS5607B MS5607alt(true);
MPU6050 IMU;
Madgwick filter;

File ADASdatafile;

void onLaunch() {
  ADAS.launched = true;
  ADAS.launch_time = millis();
  CurieIMU.detachInterrupt();
}

void ADASWDtimeout() {
  /* Executed by main watchdog timer if it times out */
    
  ADAS.inFatalError = true; // emergency stop
  MotorStop();
  ADAS.desiredpos = ADAS.pulse_count;
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

  if (ADAS.inFatalError) {
    MotorStop(); // stop motor entirely
    ADASbeep(-99); // beep like nothing else
    return;
  }

  if (ADAS.inFatalError) { //This is a fatal condition. ADAS must be reset to clear.
    ADAS.desiredpos = ADAS.pulse_count;
    ADASbeep(-99);
    return;
  }
  if (ADAS.dir == 1) {
    ADAS.pulse_count++;
  } else if (ADAS.dir == -1)  {
    ADAS.pulse_count--;
  }

  if ((ADAS.pulse_count >= (ADAS.desiredpos - ADAS_ERROR)) && (ADAS.pulse_count <= (ADAS.desiredpos + ADAS_ERROR))) {
    ADAS.dir = 0;
    digitalWrite(hbridgeIN1pin, HIGH);
    digitalWrite(hbridgeIN2pin, HIGH);
    digitalWrite(hbridgeENpin, LOW);

  }
  if (ADAS.pulse_count >= ADAS_MAX_DEPLOY) {
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
  ADAS.pulse_count = 0;
  MotorStop();
}

void ADASmove() {
  /*
    Actually moves the ADAS motor according to ADAS.desiredpos and ADAS.pulse_count.
  */
  static unsigned int lastmillis = 0;
  if (ADAS.inFatalError) {
    MotorStop();
    ADASbeep(-99);
    return;
  }

  if (ADAS.slow) {
    analogWrite(hbridgeENpin, ADAS_PWM_FREQ);
  } else {
    digitalWrite(hbridgeENpin, HIGH);
  }
  if (ADAS.dir == 0) {
    ADAS.slow = false;
  }
  MotorMove(ADAS.dir);
}


void ADASupdate() {
  /*
    Moves ADAS according to ADAS.desiredpos. Incorporates hysteresis via ADAS_ERROR
    so the motor doesn't overshoot its target position and go into violent
    oscillations.
  */

  static unsigned int lastjerk = 0;

  ADASmove(); //ADAS won't move unless ADASmove() is called here.

  /*if (ADAS.lastdir != ADAS.dir) { //Prevents sudden changes by setting slow flag.
    ADAS.jerk = true;
    ADAS.slow = true;
    lastjerk =  millis();
  }

  if ((millis() - lastjerk) >= ADAS_MAX_JERK_TIME && ADAS.jerk == true) {
    ADAS.slow = false;
    ADAS.jerk = false;
  }*/

  if (ADAS.pulse_count <= (ADAS.desiredpos - ADAS_ERROR)) { // Need to go forward to achieve target pos.
    ADAS.dir = 1;
   // ADAS.lastdir = ADAS.dir;
    if (ADAS.pulse_count >= (ADAS.desiredpos - ADAS_SLOW_THRESH)) { //slow when approaching target
      ADAS.slow = true;
    }
  } else if (ADAS.pulse_count >= (ADAS.desiredpos + ADAS_ERROR)) { // Need to go reverse to achieve target pos.
    ADAS.dir = -1;
    ADAS.lastdir = ADAS.dir;
    if ( ADAS.pulse_count <= (ADAS.desiredpos + ADAS_SLOW_THRESH)) { //slow when approaching target
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

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
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
    int16_t ax, ay, az, gx, gy, gz;
    IMU.getMotion6(&ax,
                   &ay,
                   &az,
                   &gx,
                   &gy,
                   &gz);
    ADASdatabuf[7][i] = convertRawAcceleration(ax);
    ADASdatabuf[8][i] = convertRawAcceleration(ay);
    ADASdatabuf[9][i] = convertRawAcceleration(az);
    ADASdatabuf[10][i] = convertRawGyro(gx);
    ADASdatabuf[11][i] = convertRawGyro(gy);
    ADASdatabuf[12][i] = convertRawGyro(gz);

     filter.updateIMU(
      ADASdatabuf[7][i],
      ADASdatabuf[8][i],
      ADASdatabuf[9][i],
      ADASdatabuf[10][i],
      ADASdatabuf[11][i],
      ADASdatabuf[12][i]);
    ADASdatabuf[13][i] = filter.getPitch();
    ADASdatabuf[14][i] = filter.getRoll();
    ADASdatabuf[15][i] = filter.getYaw();
    

    ADASdatabuf[16][i] = (CurieIMU.readTemperature() / 512.0 + 23);

    if (i == 0) { // The altimeter is polled once.
      ADASdatabuf[17][i] = MS5607alt.getHeightCentiMeters();
    } else {
      ADASdatabuf[17][i] = ADASdatabuf[17][i - 1];
    }
  }
}


void writeData() {
  /*
    Writes all the sensor data to SD card
  */

  // TODO: this could probably be simplified and/or be made faster by preformatting the string that is pushed to the sd card

  /* Interrupts must be disabled or the SD card
     will be corrupted upon write. The watchdog
     is still enabled to stop ADAS if the the SD
     write locks up.
  */

  ADASdatafile = open("ADASdata.txt", FILE_WRITE);

  if (ADASdatafile) {
    for (int j = 0; j < 10; j++) { //10 blocks of
      for (int i = 0; i < 18; i++) { //9 sensor outputs
        ADASdatafile.print(ADASdatabuf[i][j]);
        ADASdatafile.print(",");
      }
      ADASdatafile.print(ADAS.launched);
      ADASdatafile.print(",");
      ADASdatafile.print(ADAS.pulse_count);
      ADASdatafile.print(",");
      ADASdatafile.print(ADAS.desiredpos);
      ADASdatafile.print(",");
      ADASdatafile.print(ADAS.dir);
      ADASdatafile.print("\n");
    }
  } else {
    // if the file didn't open, print an error:
    // NOTE: this doesnt do much in the air
    Serial.println("error opening test.txt");
    ADAS.error = -9;
    ADASbeep(-9);
  }
  close(ADASdatafile);
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
}


int ADASselftest() {
  /*
    Checks the fins for proper behavior and
    verifies that h-bridge and motor
    connections are correct. Also does an
    SD card write and read test. Returns 0 if
    everything is OK. A negative if not so.
  */
  static int lastpulse_count = ADAS.pulse_count;

  for ( int i = 0; i < 1000 && lastpulse_count == ADAS.pulse_count; i++) {
    digitalWrite(hbridgeENpin, HIGH);
    digitalWrite(hbridgeIN1pin, HIGH); //test stop condition 1
    digitalWrite(hbridgeIN2pin, HIGH);
    digitalWrite(hbridgeIN1pin, LOW); //test stop condition 2
    digitalWrite(hbridgeIN2pin, LOW);
    delay(10);
  }
  if (lastpulse_count != ADAS.pulse_count) { // Stop condition falure.
    ADAS.error = -2;
    return ADAS.error;
  }
  ADAS.desiredpos = ADAS.pulse_count + ADAS_ERROR + 2; //test forward condition.
  for (int i = 0; i < 100; i++) {
    ADASupdate();
    delay(1);
  }
  if (ADAS.pulse_count < lastpulse_count) { //Polarity swapped.
    ADAS.error = -3;
    ADAS.inFatalError = true;
    return ADAS.error;
  }
  if (ADAS.pulse_count == lastpulse_count) { //Motor not moving.
    ADAS.error = -4;
    ADAS.desiredpos = 0;
    return ADAS.error;
  }

  ADAS.desiredpos = ADAS.pulse_count - ADAS_ERROR - 2; //test reverse condition.
  for (int i = 0; i < 1000; i++) {
    ADASupdate();
    delay(1);
  }
  if (ADAS.pulse_count > lastpulse_count) { //Polarity swapped.
    ADAS.error = -5;
    ADAS.desiredpos = 0;
    return ADAS.error;
  }
  if (ADAS.pulse_count == lastpulse_count) { //Motor not moving this time. Reverse is not working.
    ADAS.error = -6;
    ADAS.desiredpos = 0;
    return ADAS.error;
  }

  ADAS.desiredpos = ADAS_MAX_DEPLOY; //test full range
  for (int i = 0; i < 2000 && lastpulse_count == ADAS.pulse_count; i++) {
    ADASupdate();
    delay(10);
  }
  ADAS.desiredpos = 50;
  for (int i = 0; i < 2000 && lastpulse_count == ADAS.pulse_count; i++) {
    ADASupdate();
    delay(10);
  }
  if ((ADAS.pulse_count > (50 + ADAS_ERROR)) || (ADAS.pulse_count < (50 - ADAS_ERROR))) { //Motor indexing issue.
    ADAS.error = -7;
    return ADAS.error;
  }

  ADAS.desiredpos = 0 - ADAS_ERROR - 1;
  for (int i = 0; i < 1000 && lastpulse_count == ADAS.pulse_count; i++) {
    ADASupdate();
    delay(10);
  }
  if ((ADAS.pulse_count > (0 + ADAS_ERROR)) || (ADAS.pulse_count < (0 - ADAS_ERROR))) { //Motor past limit. Fins disengaged or limit switch failure.
    ADAS.error = -8;
    ADAS.inFatalError = true;
    return ADAS.error;
  }

  char ADASteststring[11] = "TESTING123";
  File ADAStestfile;

  ADAStestfile = open("ADAStestfile.txt", FILE_WRITE);
  ADAStestfile.println(ADASteststring);
  close(ADAStestfile);

  ADAStestfile = open("ADAStestfile.txt", FILE_READ);
  char readtestbuf[11];
  for (int i = 0; i < 11 && ADASdatafile.available(); i++) {
    if (ADAStestfile.read() != ADASteststring[i]) {
      ADAS.error = -9;  //File read/write error.
      close(ADAStestfile); // close even if there is an error
      return ADAS.error;
    }
  }
  close(ADAStestfile);

  ADAS.error = 0;
  return ADAS.error; // All tests passed.
}

void AttachInterrupts() {
  /*
    All interrupts that should be detached for sd card reading/writting
  */
    attachInterrupt(digitalPinToInterrupt(limitswitchpin), ADASzero, FALLING); //Catch interrupts from the encoder.
  attachInterrupt(digitalPinToInterrupt(encoderpinA), ADASpulse, RISING); //Catch interrupts from the encoder.
  CurieIMU.interrupts(CURIE_IMU_SHOCK);
}

void DetachInterrupts() {
  /*
    All interrupts that should be reatached after sd card reading/writting
  */
  detachInterrupt(encoderpinA);
  CurieIMU.noInterrupts(CURIE_IMU_SHOCK);
  // don't detach watchdog to catch failure in sd card writing
}

File open(char filename[]) {
  /*
    makes open consistent with the implementation in the SD library, where the default mode is FILE_READ
  */
  return open(filename, FILE_READ);
}

File open(char filename[], byte mode) {
  /*
    Detaches interrupts while file is open to avoid corruption of file
  */

  // NOTE: motor overshoot can happen
  MotorStop();
  DetachInterrupts();
  return SD.open(filename, mode);
}

void close(File file) {
  /*
    reattaches interrupts after closing the file to avoid corruption of the file in question
  */
  file.close();
  AttachInterrupts();
  MotorMove(ADAS.dir);
}

void MotorStop() {
  MotorMove(0);
}

void MotorMove(int direction) {
  /*
    moves motor in a direction given by the direction flag
    -1 = backwards
    0 = stop
    1 = forwards
  */
    digitalWrite(hbridgeENpin, HIGH);

  if (direction == 1) { //forward
    digitalWrite(hbridgeIN1pin, HIGH);
    digitalWrite(hbridgeIN2pin, LOW);
  } else if (direction == -1) { //reverse
    digitalWrite(hbridgeIN1pin, LOW);
    digitalWrite(hbridgeIN2pin, HIGH);
  } else if (direction == 0) { // STOP
    digitalWrite(hbridgeIN1pin, HIGH);
    digitalWrite(hbridgeIN2pin, HIGH);
  }
}


void setup() {
//  BLESerial.setName("ADAS");
//  BLESerial.begin();

  Serial.begin(9600);

  /* For the altimeter */
  MS5607alt.init();

  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 75);
  /* For the built-in Curie IMU */
  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(16);
  CurieIMU.setGyroRate(3200);
  CurieIMU.setAccelerometerRate(12.5);

  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 300);
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 75);
  CurieIMU.attachInterrupt(onLaunch);
  
  
  IMU.initialize();
  IMU.setRate(1600);

  filter.begin(1600);

  while (!SD.begin(sdpin)) { //Stop everything if we cant see the SD card!
    Serial.println("Card failed or not present.");
    ADASbeep(-1);
    // delay(1000); already blocked for at least 1 second by the beeping
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

        digitalWrite(hbridgeENpin, HIGH);


  AttachInterrupts();

  /* Run self-test until pass. */
//  while (ADAS.error != 0) {
//    ADASbeep(ADASselftest());
//  }

  //CurieTimerOne.start(WDUS, &ADASWDtimeout); //Starts watchdog timer.
}


void loop() {
  //CurieTimerOne.restart(WDUS); //Restarts watchdog timer.
  //BLESerial.println("Hello, this is ADAS.");
  getData();
  writeData();
  if (ADAS.launched) {
    if (25000 > (millis() - ADAS.launch_time) && (millis() - ADAS.launch_time) > 10000 && ADAS.desiredpos == 0) {
      ADAS.desiredpos = 100;
    } else if (25000<(millis() - ADAS.launch_time) && ADAS.desiredpos != 0) {
      ADAS.desiredpos = 0;
    }
  }
  ADASupdate();
  //ADASlaunchtest();
//  Serial.println(ADAS.launched);
//  Serial.println(ADAS.error);
}

