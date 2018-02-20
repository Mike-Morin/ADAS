/*
    Complete refactor of the ADAS_feb05a sketch to with less loops and more async.
    Most hard work is done by interrupts and the rest is left to 
*/

#include "MS5607/IntersemaBaro.h"
#include <CurieTimerOne.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <CurieIMU.h>

const int ADAS_MAX_POSITION = 220; // (steps) max number of steps that adas can be away from 0
const int ADAS_NORMAL_PWM = 255; // (#/255) normal pwm
const int ADAS_SLOW_PWM = 75; // (#/255) pwm of slow mode
const int CREEPUP_THRESHOLD = 10; // (steps) number of steps away from ADAS's target position where slow mode is engaged
const int LAUNCH_THRESHOLD_ACC = 4; // (G) amount of acceleration needed to trip the launch detection

const int WATCHDOG_LIMIT = 1000000;

/* pin definitions */
const int hbridgeIN1_pin = 2;
const int hbridgeIN2_pin = 6;
const int hbridgeEN_pin = 5;
const int limitswitch_pin = 4;
const int beeper_pin = 3;
const int encoderA_pin = 7;
const int sd_pin = 10;

/*
    pin defs in the SPI library
    SS    -- pin 10
    MOSI  -- pin 11
    MISO  -- pin 12
    SCK   -- pin 13
*/

// function and type definitions

// directions that the motor is allowed to go
const int FORWARD = 1;
const int STOP = 0;
const int REVERSE = -1;



typedef struct {
    bool launched = false;
    bool descending = false; // for future use: why continue running adas when the rocket is falling?
    bool inError = false;
    bool slowmode = false;
    bool enforce_target = true;

    volatile int target = 0;
    volatile int position = 0;
    volatile int error = 0;
    volatile int direction = 0;
} ADASstate;

// data buffer struct, allows you to easily (and relatively painlessly) add more sensors and such
typedef struct {
    int length;
    uint64_t ts[10];
    float accelerometer[10][3]; // 3 axis
    float gyroscope[10][3]; // 3 axis
    float temperature[10];
    float altimeter[10];
    bool launched[10];
    int position[10];
    bool target[10];
    int direction[10];
} DataBuffer;

ADASstate ADAS;
DataBuffer DB;
Intersema::BaroPressure_MS5607B MS5607alt(true);


// interrupt driven functions
void WatchdogTimeout() {
    /*
        Executed when the watchdog times out
    */
    ADAS.inError = true;
    ADAS.error = -99;
    SetMotorDirection(STOP);
    beep(ADAS.error);
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
        ADAS.target = 0; // retract aerobreak for safe landings
        SetMotorDirection(REVERSE);
    }
}

void onEncoderPulse() {
    /* 
        Executed when encoder pulse is recieved
    */

    ADAS.position = ADAS.position+ADAS.direction;
    // always increment position
    if (ADAS.enforce_target) { // if we arent testing
        if ( ADAS.target < ADAS_MAX_POSITION && ADAS.target > 0) {
            // set direction of motor 
            if (ADAS.position < ADAS.target && ADAS.direction != FORWARD) {
                SetMotorDirection(FORWARD);
            } else if (ADAS.position > ADAS.target && ADAS.direction != REVERSE) {
                SetMotorDirection(REVERSE);
            } else {
                SetMotorDirection(STOP);
            }

            if (!ADAS.slowmode && abs(ADAS.target - ADAS.position) < CREEPUP_THRESHOLD) {
                SetMotorSpeed(ADAS_SLOW_PWM);
                ADAS.slowmode = true;
            } else if (ADAS.slowmode && abs(ADAS.target - ADAS.position) > CREEPUP_THRESHOLD) {
                SetMotorSpeed(ADAS_NORMAL_PWM);
                ADAS.slowmode = false;
            }
        } else {
            if (ADAS.target > ADAS_MAX_POSITION) { // if target is set higher than allowed
                ADAS.target = ADAS_MAX_POSITION;
            } else if (ADAS.target < 0) {
                ADAS.target = 0;
            }
        }
    }
}

void onLimitReached() {
    /*
        When minimum limit reached
    */
    
    // zero the position
    ADAS.position = 0;
    // stop the motors
    SetMotorDirection(STOP);
}

// normal functions
void SetMotorDirection(int direction) {
    /* 
        sets the ADAS motor direction
    */
    if (direction == FORWARD) { //forward
        digitalWrite(hbridgeIN1_pin, HIGH);
        digitalWrite(hbridgeIN2_pin, LOW);
    } else if (direction == REVERSE) { //reverse
        digitalWrite(hbridgeIN1_pin, LOW);
        digitalWrite(hbridgeIN2_pin, HIGH);
    } else if (direction == STOP) { // STOP
        digitalWrite(hbridgeIN1_pin, HIGH);
        digitalWrite(hbridgeIN2_pin, HIGH);
    }
}

void SetMotorSpeed(int speed) {
    /*
        Sets the adas motor speed to speed (#/255)
    */
    analogWrite(hbridgeEN_pin, speed);
}

float AccelerationMagnitude(float vector[]) {
    /*
        calculates the magnitude of the acceleration from an array of x, y and z
    */
    return AccelerationMagnitude(vector[0], vector[1], vector[2]);
}

float AccelerationMagnitude(float xAcc, float yAcc, float zAcc) {
    /*
        gets the magnetude of the acceleration on the accelerometer
    */
    return sqrt(pow(xAcc, 2)+pow(yAcc, 2)+pow(zAcc,2));
}

void GetData() {
    /*
        Polls all sensors and puts them into the data buffer
    */
    if (DB.length == 10) {
        DB.length = 0; // reset adas data buffer once its been filled
    }

    DB.altimeter[DB.length] = MS5607alt.getHeightCentiMeters();
    CurieIMU.readAccelerometerScaled(
        DB.accelerometer[DB.length][0],
        DB.accelerometer[DB.length][1],
        DB.accelerometer[DB.length][2]
    );
    CurieIMU.readGyroScaled(
        DB.gyroscope[DB.length][0],
        DB.gyroscope[DB.length][1],
        DB.gyroscope[DB.length][2]
    );
    DB.temperature[DB.length] = (CurieIMU.readTemperature()/512.0)+23;
    DB.launched[DB.length] = ADAS.launched;
    DB.position[DB.length] = ADAS.position;
    DB.target[DB.length] = ADAS.target;
    DB.direction[DB.length] = ADAS.direction;
    DB.ts[DB.length] = millis(); // doubles as a way to tell how long each loop takes
    DB.length++;

}

void WriteData() {
    /*
        Writes all data to disk
        format : <time_since_start>,<accX>,<accY>,<accZ>,<gyroX>,<gyroY>,<gyroZ>,<temp>,<alt>,<launched>,<pos>,<target>,<direction>

        TODO: do this all in one string format, might be faster...
    */

    File dataFile = open("ADASdata.txt", FILE_WRITE);

    if (dataFile) {
        for (int i=0; i < DB.length; i++) { // only write new data to the file
            dataFile.print(DB.ts[i]);
            dataFile.print("\t");
            for (int j=0; j < 3; j++) {
                dataFile.print(DB.accelerometer[i][j]);
                dataFile.print("\t");
            }
            for (int j=0; j < 3; j++) {
                dataFile.print(DB.gyroscope[i][j]);
                dataFile.print("\t");
            }
            dataFile.print(DB.temperature[i]);
            dataFile.print("\t");
            dataFile.print(DB.altimeter[i]);
            dataFile.print("\t");
            dataFile.print(DB.launched[i]);
            dataFile.print("\t");
            dataFile.print(DB.position[i]);
            dataFile.print("\t");
            dataFile.print(DB.target[i]);
            dataFile.print("\t");
            dataFile.println(DB.direction[i]);
        }
    }
    close(dataFile);
}

void AttachInterrupts() {
    attachInterrupt(digitalPinToInterrupt(encoderA_pin), onEncoderPulse, RISING);
    attachInterrupt(digitalPinToInterrupt(limitswitch_pin), onLimitReached, FALLING); // falling edge because when the limit swich is  normally closed
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
    detachInterrupt(encoderA_pin);
    detachInterrupt(limitswitch_pin);
    CurieIMU.detachInterrupt();
}

int ADASSelfTest() {
    /*
        Selftest program run ever time it initializes
        must be run after interrupts are set
    */
    digitalWrite(hbridgeEN_pin, HIGH);

    ADAS.enforce_target = false;

    int pc = ADAS.position;
    // test stop mode 1 (both pins high)
    digitalWrite(hbridgeIN1_pin, HIGH);
    digitalWrite(hbridgeIN2_pin, HIGH);
    delay(500);

    if (pc != ADAS.position) {
        return -2; // stop mode 1 did not work as intended
    }
    // test stop mode 2 (both pins low)
    digitalWrite(hbridgeIN1_pin, LOW);
    digitalWrite(hbridgeIN2_pin, LOW);
    delay(500);

    if (pc != ADAS.position) {
        return -3; // stop mode 1 did not work as intended
    }
    // check if motor can go forward
    // pc should not have changed yet
    SetMotorDirection(FORWARD);
    delay(100);

    if (pc < ADAS.position) {
        return -4; // motor went reverse instead of forwards
    } else if (pc == ADAS.position) {
        return -5; // motor didn't move
    }

    SetMotorDirection(STOP);
    pc = ADAS.position;
    delay(100);

    if (pc != ADAS.position) {
        return -6; // motor did not stop
    }

    SetMotorDirection(REVERSE);
    delay(100);
    SetMotorDirection(STOP);
    if (pc > ADAS.position ) {
        return -7; // motor went forwards instead of reverse
    } else if (pc == ADAS.position) {
        return -8; // motor didn't move when in reverse
    }

    ADAS.enforce_target = true; // reset adas.enforce target


    ADAS.target = 220;
    SetMotorDirection(FORWARD);
    delay(1000); // wait until ADAS should be fully extended
    SetMotorDirection(STOP);
    if (ADAS.position > ADAS.target) {
        return -9; // over shot on full extend
    } else if (ADAS.position < ADAS.target) {
        return -10; // under shot on full extend
    }

    ADAS.target = 0;
    SetMotorDirection(REVERSE);
    delay(1000);
    SetMotorDirection(STOP);

    if (ADAS.position > ADAS.target) {
        return -11; // undershot on full retract
    } else if (ADAS.position < ADAS.target) {
        return -12; // overshot on full retract
    }

    ADAS.target = 1000;
    SetMotorDirection(FORWARD);
    delay(1500);
    SetMotorDirection(STOP);
    if (ADAS.position != ADAS_MAX_POSITION) {
        return -13; // failed to stop at max position
    }

    ADAS.target = -1000;
    SetMotorDirection(REVERSE);
    delay(2000);
    SetMotorDirection(STOP);

    if (ADAS.position != 0) {
        return -14; // failed to stop at limitswitch
    }

    char testString[] = "test string 123";

    File testfile = open("testfile.txt", FILE_WRITE);
    testfile.println(testString);
    close(testfile);

    testfile = open("testfile.txt", FILE_READ);
    for (int i = 0; i < 15; i++) {
        if (testfile.read() != testString[i]) {
            return -15; // sd card error
        }
    }
    close(testfile);
    return 0;
}

File open(char filename[], byte mode) {
    SetMotorSpeed(STOP); // Stop motors
    DetachInterrupts(); // stop interupts
    SetMotorSpeed(STOP); // Make sure no fuckery happend while waiting for interrupts to detach or something
    return SD.open(filename, mode); // open file for reading
}

void close(File file) {
    file.close();
    AttachInterrupts(); // reattach the interrupts 
    onEncoderPulse(); // sorts everything out based on where the position is and where the target position is
}

void beep(int code) {
    /*
    Plays informational beeps, codes are integers, negative codes are errors,
    positive codes are purely informational
    error code -99 is an extreme error and is indicated by a constant tone
  */

  if (code == -99) { //Motor emergency stop. Beep forever.
    digitalWrite(beeper_pin, HIGH);
    while (true) {} // really do it forever until rebooted
  }

  if (code < 0) {
    // Critical Errors are negative
    // if there is a critical error there is a 2 second long beep before the code
    digitalWrite(beeper_pin, HIGH);
    delay(2000);
  } else {
    // any non negative code is a information code
    // there is a 400ms long beep before these codes
    digitalWrite(beeper_pin, HIGH);
    delay(400);
  }
  digitalWrite(beeper_pin, LOW);
  delay(1000);
  // beep out the code
  for (int i = 0; i < abs(code); i++) {
    digitalWrite(beeper_pin, HIGH);
    delay(200);
    digitalWrite(beeper_pin, LOW);
    delay(200);
  }
}

void setup() {
    Serial.begin(9600);

    MS5607alt.init();
    CurieIMU.begin();
    CurieIMU.setAccelerometerRange(16);
    CurieIMU.setGyroRate(3200);
    CurieIMU.setAccelerometerRate(1600);

    CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 9.81*LAUNCH_THRESHOLD_ACC*1000); // amount that counts as a launch
    CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 75); // constant upwards acceleration for at least 75 ms (max)

    CurieIMU.setDetectionThreshold(CURIE_IMU_FREEFALL, 3.91);
    
    while (!SD.begin(sd_pin)) {
        beep(-1); // sd card not working
    }
    beep(1); // sd card working

    pinMode(hbridgeIN1_pin, OUTPUT);
    pinMode(hbridgeIN2_pin, OUTPUT);
    pinMode(hbridgeEN_pin, OUTPUT);
    pinMode(encoderA_pin, INPUT);
    pinMode(limitswitch_pin, INPUT);

    AttachInterrupts();

    int error = ADASSelfTest();
    Serial.print("Self Test Error: ");
    Serial.println(error);
    while (error != 0) {
        beep(error);
    }

    CurieTimerOne.start(WATCHDOG_LIMIT, &WatchdogTimeout);
    CurieIMU.interrupts(CURIE_IMU_SHOCK);
    CurieIMU.attachInterrupt(onLaunch);
    // may need to stop interrupts on FIFO full and data ready
}


void loop() {
    CurieTimerOne.restart(WATCHDOG_LIMIT);
    GetData();
    if (DB.length == 10) {
        WriteData();
    }
    if (!ADAS.launched) {
        /*
            Things to repeat prelaunch
        */
    } else if (ADAS.launched && !ADAS.descending) {
        /*
            Things to repeat during flight
        */
    } else {
        /*
            Things to do while descending or on the ground
        */

    }
}

