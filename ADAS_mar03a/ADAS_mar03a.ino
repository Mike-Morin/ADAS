#include "MS5607/IntersemaBaro.h"

#include <CurieTimerOne.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h> // Magic IMU positioning angle library
#include <math.h> // MATH
#include "MPU6050.h" // External, nicer altimeter used as the data source


#define ADAS_ERROR 5 //Allowed error in ADAS motion due to overshoot. (encoder pulses) 
#define ADAS_SLOW_THRESH 10 // Number of pulses away from target at which ADAS slows down. (encoder pulses)
#define ADAS_MAX_JERK_TIME 100 // Amount of time ADAS slowing is to be applied before jerk condition is cleared. (milliseconds)
#define ADAS_MAX_DEPLOY 220 // Max number of ADAS pulses--this is where the fins disengage.(encoder pulses)
#define ADAS_PWM_FREQ 150 // PWM frequency for slow condition. (#/255)
#define LAUNCH_THRESHOLD_TIME 200 //(ms)
#define LAUNCH_THRESHOLD_ACC 4 //(g)

const int DATABUFFER_LENGTH = 10;

const int IMU_SAMPLE_RATE = 3200; // (hz)
const int IMU_ACCEL_RANGE = 16; // (g)

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
    bool descending = false;
} ADASstate;

ADASstate ADAS;

typedef struct {
    unsigned long ts; // time stamp
    float OB_accel[3]; // on board accelerometer
    float OB_gyro[3]; // on board gyroscope
    int16_t EX_accel[3]; // external (of board) accelerometer (MPU6050) (mg/s)
    int16_t EX_gyro[3]; // external (of board) gyroscope (MPU6050)  (mg/s)
    float angle[3];
    float velocity;
    float position;
    float vertical_velocity;
    float altimeter;
    float temperature;
    bool launched;
    bool descending;
    int adas_target;
    int adas_position;
} dataframe;

/* Data variables */

dataframe databuffer[DATABUFFER_LENGTH];
int current_index = 0;

Intersema::BaroPressure_MS5607B MS5607alt(true);

MPU6050 IMU;
Madgwick filter;

File ADASdatafile;

void AttachInterrupts(){
	attachInterrupt(digitalPinToInterrupt(encoderpinA), ADASpulse, RISING); //Catch interrupts from the encoder.
	attachInterrupt(digitalPinToInterrupt(limitswitchpin), ADASzero, FALLING); // catch when the limit switch is disengaged

	if (!ADAS.launched) {
        if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
            onLaunch();
        } else {
            CurieIMU.attachInterrupt(onLaunch);
        }
    } else if (ADAS.launched && !ADAS.descending) {
        if (CurieIMU.getInterruptStatus(CURIE_IMU_FREEFALL)) {
            onApogee();
        } else {
            CurieIMU.attachInterrupt(onApogee);
        }
    }
}

void DetachInterrupts() {
    detachInterrupt(encoderpinA);
    detachInterrupt(limitswitchpin);
    CurieIMU.detachInterrupt();
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void onLaunch() {
    if (!ADAS.launched) {
        ADAS.launched = true;
        // remove launch interrupt
        CurieIMU.detachInterrupt();
        CurieIMU.noInterrupts(CURIE_IMU_SHOCK);
        // add apogee interrupt
        CurieIMU.interrupts(CURIE_IMU_FREEFALL);
        CurieIMU.attachInterrupt(onApogee);
    }
}

void onApogee() {
    if (!ADAS.descending) {
        ADAS.descending = true;
        ADAS.desiredpos = 0; // retract aerobreak for safe landings
    	CurieIMU.detachInterrupt();
    }
}


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

  if (ADAS.lastdir != ADAS.dir && ADAS.jerk == false) { //Prevents sudden changes by setting slow flag.
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





void log_data() {

    dataframe current_frame;

    current_frame.ts = millis();
    current_frame.altimeter = MS5607alt.getHeightCentiMeters(); // TODO: scale to meters??
    CurieIMU.readAccelerometerScaled(
       current_frame.OB_accel[0],
       current_frame.OB_accel[1],
       current_frame.OB_accel[2]
    );
    CurieIMU.readGyroScaled(
        current_frame.OB_gyro[0],
        current_frame.OB_gyro[1],
        current_frame.OB_gyro[2]
    );

    IMU.getMotion6(
        &current_frame.EX_accel[0],
        &current_frame.EX_accel[1],
        &current_frame.EX_accel[2],
        &current_frame.EX_gyro[0],
        &current_frame.EX_gyro[1],
        &current_frame.EX_gyro[2]
    );

    filter.updateIMU(
        convertRawAcceleration(current_frame.EX_accel[0]),
        convertRawAcceleration(current_frame.EX_accel[1]),
        convertRawAcceleration(current_frame.EX_accel[2]),
        convertRawGyro(current_frame.EX_gyro[0]),
        convertRawGyro(current_frame.EX_gyro[1]),
        convertRawGyro(current_frame.EX_gyro[2])
    );

    

    current_frame.angle[0] = filter.getPitch();
    current_frame.angle[1] = filter.getRoll();
    current_frame.angle[2] = filter.getYaw();

    current_frame.velocity = getVelocity();

    current_frame.temperature = (CurieIMU.readTemerature()/512.0)+23;

    databuffer[current_index] = current_frame;
    current_index++;
    if (current_index == DATABUFFER_LENGTH) {
        current_index = 0;
    }
}

void write_data() {
    File datafile = open("ADAS_DATA.txt", FILE_WRITE);

    if (datafile) {
        for (int i=0; i<DATABUFFER_LENGTH; i++){
            dataframe current_frame = databuffer[i];
            char buffer[256];
            sprintf(buffer, 
                "%xl,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%i,%i,%i",
                current_frame.ts,
                current_frame.OB_accel[0],
                current_frame.OB_accel[1],
                current_frame.OB_accel[2],
                current_frame.OB_gyro[0],
                current_frame.OB_gyro[1],
                current_frame.OB_gyro[2],
                current_frame.EX_accel[0],
                current_frame.EX_accel[1],
                current_frame.EX_accel[2],
                current_frame.EX_gryo[0],
                current_frame.EX_gryo[1],
                current_frame.EX_gryo[2],
                current_frame.angle[0],
                current_frame.angle[1],
                current_frame.angle[2],
                current_frame.velocity,
                current_frame.altimeter,
                current_frame.temperature,
                current_frame.launched,
                current_frame.descending,
                current_frame.adas_target,
                current_frame.adas_position
            );
            datafile.write(buffer);
        }
        datafile.close();
    } else {
        datafile.close(); // can never be too careful
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
  int lastpulsect = ADAS.pulsect;

  for ( int i = 0; i < 100 && lastpulsect == ADAS.pulsect; i++) {
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
    delay(10);
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

  ADAS.desiredpos = ADAS.pulsect - (ADAS_ERROR - 2) * 2; //test reverse condition.
  for (int i = 0; i < 100; i++) {
    ADASupdate();
    delay(10);
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
  for (int i = 0; i < 200 && lastpulsect == ADAS.pulsect; i++) {
    ADASupdate();
    delay(10);
  }
  ADAS.desiredpos = 50;
  for (int i = 0; i < 200 && lastpulsect == ADAS.pulsect; i++) {
    ADASupdate();
    delay(10);
  }
  if ((ADAS.pulsect > (50 + ADAS_ERROR)) || (ADAS.pulsect < (50 - ADAS_ERROR))) { //Motor indexing issue.
    ADAS.error = -7;
    return ADAS.error;
  }

  ADAS.desiredpos = 0 - ADAS_ERROR - 1;
  for (int i = 0; i < 100 && lastpulsect == ADAS.pulsect; i++) {
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
	
  Serial.begin(9600);
  
    /* ADAS control stuff */
  pinMode(hbridgeIN1pin, OUTPUT); //hbridge IN1
  pinMode(hbridgeIN2pin, OUTPUT); //hbridge IN2
  pinMode(hbridgeENpin, OUTPUT); //hbridge EN pin for pwm
  pinMode(encoderpinA, INPUT); //encoder A (or B... either works).
  pinMode(limitswitchpin, INPUT);

  /* For the altimeter */
  MS5607alt.init();

  /* For the built-in Curie IMU */
  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(16);
  CurieIMU.setGyroRate(3200);
  CurieIMU.setAccelerometerRate(1600);
  
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 9.81*LAUNCH_THRESHOLD_ACC*1000); // amount that counts as a launch
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 75); // constant upwards acceleration for at least 75 ms (max)
  CurieIMU.setDetectionThreshold(CURIE_IMU_FREEFALL, 3.91);

  // configure better, external imu
  IMU.initialize();
  IMU.setRate(IMU_UPDATE_RATE);
  
  filter.begin(IMU_UPDATE_RATE);
  
  // configure micros per reading and previous micros
	microsPerReading = 1000000/IMU_UPDATE_RATE;
	microsPrevious = micros();
    
  while (!SD.begin(sdpin)) { //Stop everything if we cant see the SD card!
    Serial.println("Card failed or not present.");
    ADASbeep(-1);
    delay(1000);
  }

  Serial.println("Card OK");
  ADASbeep(1);






  /* Run self-test until pass. */
  while (ADAS.error != 0) {
    ADASbeep(ADASselftest());
  }

  CurieTimerOne.start(WDUS, &ADASWDtimeout); //Starts watchdog timer.
}


void loop() {
  CurieTimerOne.restart(WDUS); //Restarts watchdog timer.
  log_data();
  write_data();
  ADASupdate();
  ADASlaunchtest();
  Serial.println(ADAS.launched);
  Serial.println(ADAS.error);


}
