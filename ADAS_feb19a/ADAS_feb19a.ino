/*
    Complete refactor of the ADAS_feb05a sketch to with less loops and more async.
    Most hard work is done by interrupts and the rest is left to
*/

#include "MS5607/IntersemaBaro.h" // barometric altimeter
#include <CurieTimerOne.h> // curie watchdog timer
#include <SPI.h> // spi library
#include <SD.h> // SD card library
#include <Wire.h> // wire library
#include <CurieIMU.h> // Internal IMU library
#include <MadgwickAHRS.h> // Magic IMU positioning angle library
#include <math.h> // MATH
#include "MPU6050.h" // External, nicer altimeter used as the data source

const int ADAS_MAX_POS = 220; // (steps) max number of steps that adas can be away from 0
const int ADAS_NORMAL_PWM = 255; // (#/255) normal pwm
const int CREEPUP_THRESHOLD = 10; // (steps) number of steps away from ADAS's target position where slow mode is engaged
const int LAUNCH_THRESHOLD_ACC = 4; // (G) amount of acceleration needed to trip the launch detection
const int IMU_UPDATE_RATE = 25; // (hz) number of times the imu is read per second

const int IMU_ACC_RANGE = 2; // (g) accelerometer range for

const int WATCHDOG_LIMIT = 1000000; // timeout in miliseconds
const int ADAS_POS_ERROR = 5; // (steps) number of motor overshoot steps allowed.

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

enum dir {
  FORWARD,
  REVERSE,
  STOP
};



typedef struct {
    bool launched = false;
    bool descending = false; // for future use: why continue running adas when the rocket is falling?
    bool inError = false;
    bool slowmode = false;
    bool enforce_target = true;

    volatile int target = 0;
    volatile int position = 0;
    volatile int error = 0;
    dir direction = STOP;
} ADASstate;

// data buffer struct, allows you to easily (and relatively painlessly) add more sensors and such
typedef struct {
    int length;
    unsigned long ts[10];
    float accelerometer[10][3]; // 3 axis
    float gyroscope[10][3]; // 3 axis
    float angle[10][3];
    float velocity[10];
    float position[10];
    float vertical_velocity[10];
    float altimeter[10];
    float temperature[10];
    bool launched[10];
    int target[10];
    dir direction[10];
} DataBuffer;

ADASstate ADAS;
DataBuffer DB;
Intersema::BaroPressure_MS5607B MS5607alt(true);
MPU6050 IMU;
Madgwick filter;

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
    	CurieIMU.detachInterrupt();
    }
}

void onEncoderPulse() {
    /*
        Executed when encoder pulse is recieved
    */

    switch (ADAS.direction) {
      case FORWARD:
        ADAS.position++;
        break;
      case REVERSE:
        ADAS.position--;
        break;
      case STOP:
        SetMotorDirection(STOP);
        break;
    }
    

    ADAS.position = ADAS.position+ADAS.direction;
    // always increment position
    if (ADAS.enforce_target) { // if we arent testing
        if ( ADAS.target < ADAS_MAX_POS && ADAS.target > 0) {

            // set direction of motor
            if (ADAS.position < (ADAS.target - ADAS_POS_ERROR) && ADAS.direction != FORWARD) {
                SetMotorDirection(FORWARD);
            } else if (ADAS.position > (ADAS.target + ADAS_POS_ERROR) && ADAS.direction != REVERSE) {
                SetMotorDirection(REVERSE);
            } else if (ADAS.position > (ADAS.target - ADAS_POS_ERROR) &&  ADAS.position < (ADAS.target + ADAS_POS_ERROR)){
                SetMotorDirection(STOP);
            }
        } else {
            SetMotorDirection(STOP);
            if (ADAS.target > ADAS_MAX_POS) { // if target is set higher than allowed
                ADAS.target = ADAS_MAX_POS;
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
    //Serial.println("RIP");
    // zero the position
    //ADAS.position = 0;
    // stop the motors
    //SetMotorDirection(STOP);
}

// normal functions
void SetMotorDirection(dir foo) {
    /*
        sets the ADAS motor direction
    */
    if (foo == FORWARD) { //forward
        ADAS.direction = FORWARD;
        digitalWrite(hbridgeEN_pin, HIGH);
        digitalWrite(hbridgeIN1_pin, HIGH);
        digitalWrite(hbridgeIN2_pin, LOW);
	      return;
    } else if (foo == REVERSE) { //reverse
        ADAS.direction = REVERSE;
        digitalWrite(hbridgeEN_pin, HIGH);
        digitalWrite(hbridgeIN1_pin, LOW);
        digitalWrite(hbridgeIN2_pin, HIGH);
	return;
    } else if (foo == STOP) { // STOP
        ADAS.direction = STOP;
        digitalWrite(hbridgeIN1_pin, HIGH);
        digitalWrite(hbridgeIN2_pin, HIGH);
	return;
    }
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

    DB.altimeter[DB.length-1] = MS5607alt.getHeightCentiMeters();
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
    filter.updateIMU(
        DB.gyroscope[DB.length][0],
        DB.gyroscope[DB.length][1],
        DB.gyroscope[DB.length][2],
        DB.accelerometer[DB.length][0],
        DB.accelerometer[DB.length][1],
        DB.accelerometer[DB.length][2]
    );
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
    attachInterrupt(digitalPinToInterrupt(limitswitch_pin), onLimitReached, FALLING); // falling edge because when the limit swich is active low
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
/*
int ADASSelfTest() {
    
        //Selftest program run ever time it initializes
        //must be run after interrupts are set
    
    digitalWrite(hbridgeEN_pin, HIGH);

    ADAS.enforce_target = false;

    int pc = ADAS.position;
    // test stop mode 1 (both pins high)
    digitalWrite(hbridgeIN1_pin, HIGH);
    digitalWrite(hbridgeIN2_pin, HIGH);
    delay(1000);

    if (pc != ADAS.position) {
        return -2; // stop mode 1 did not work as intended
    }
    // test stop mode 2 (both pins low)
    digitalWrite(hbridgeIN1_pin, LOW);
    digitalWrite(hbridgeIN2_pin, LOW);
    delay(1000);

    if (pc != ADAS.position) {
        return -3; // stop mode 1 did not work as intended
    }
    // check if motor can go forward
    // pc should not have changed yet
    
    ADAS.enforce_target = true;

    ADAS.position = 200;
    SetMotorDirection(REVERSE);
    delay(2000);

    if (ADAS.position != 0) {
      return -4; // Zeroing problem
    }
    
/*
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
    */
    /*
    return 0;
}
*/

File open(char filename[], byte mode) {
    SetMotorDirection(STOP); // Stop motors
    DetachInterrupts(); // stop interupts
    SetMotorDirection(STOP); // Make sure no fuckery happend while waiting for interrupts to detach or something
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

float g;
unsigned long microsPerReading, microsPrevious; // number of micros per reading that are required and the micros of the previous reading

void setup() {
    Wire.begin();
    Serial.begin(9600);

    MS5607alt.init();

    // configure onboard IMU, used for detecting launch and apogee only,
    CurieIMU.begin();
    CurieIMU.setAccelerometerRange(16);
    CurieIMU.setAccelerometerRate(1600);

    CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 9.81*LAUNCH_THRESHOLD_ACC*1000); // amount that counts as a launch
    CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 75); // constant upwards acceleration for at least 75 ms (max)

    CurieIMU.setDetectionThreshold(CURIE_IMU_FREEFALL, 3.91);

    // configure better, external imu
    IMU.initialize();
    IMU.setRate(IMU_UPDATE_RATE);

    //filter.begin(IMU_UPDATE_RATE);

    // configure micros per reading and previous micros
    microsPerReading = 1000000/IMU_UPDATE_RATE;
    microsPrevious = micros();

    ADAS.launched = true;

   /* while (!SD.begin(sd_pin)) {
        beep(-1); // sd card not working
    }
    beep(1); // sd card working*/

    pinMode(hbridgeIN1_pin, OUTPUT);
    pinMode(hbridgeIN2_pin, OUTPUT);
    pinMode(hbridgeEN_pin, OUTPUT);
    pinMode(encoderA_pin, INPUT);
    pinMode(limitswitch_pin, INPUT);

    AttachInterrupts();
/*
    while (true) {
        Serial.print("Self Test Error: ");
        Serial.println(ADASSelfTest());
        Serial.println(ADAS.position);
    }
    */

    // use internal Curie stuff for interupts for speed and ease of operation

    //CurieTimerOne.start(WATCHDOG_LIMIT, &WatchdogTimeout);
    CurieIMU.interrupts(CURIE_IMU_SHOCK);
    CurieIMU.attachInterrupt(onLaunch);
    // may need to stop interrupts on FIFO full and data ready
}

unsigned long loopcount = 0;

//Global variables
unsigned long prev_time = 0; //measured in seconds

void loop() {
    Serial.println("Yay");
    CurieTimerOne.restart(WATCHDOG_LIMIT);
    GetData(); // always collect data
    
    if (!ADAS.launched) {
        if (loopcount % 20 == 0) { // collect data at half speed on the launch pad
          if (DB.length == 10) {
              WriteData();
          }
        }
        /*
            Things to repeat prelaunch
        */
    } else if (ADAS.launched && !ADAS.descending) {
        /*
        Things to do during flight upwards
        */
        float height = 200;
        float velocity = 220;
        
        Serial.println("ADAS pos: ");
        Serial.println(ADAS.position);
        //record sensor data
        WriteData();
        //update the ADAS position
        //float height = DB.position[DB.length-1];
        //float velocity = DB.vertical_velocity[DB.length-1];
        /////FOR TESTING
        float prev_deployment = ADAS.target/ADAS_MAX_POS;  //the ratio of the previous deployment to full deployment
        float cur_time = 0;
        float prev_time =0;
        
        if(DB.length == 1){
          cur_time = DB.ts[0];
          prev_time = DB.ts[9];
        } else{
          cur_time = DB.ts[DB.length-1];
          prev_time = DB.ts[DB.length-2];
        }

        float time_diff = cur_time - prev_time;
        double new_ADAS_deployment = PID(height, velocity, prev_deployment, time_diff);
        //update the actual ADAS deployment
        
        ADAS.target = (int) (new_ADAS_deployment*ADAS_MAX_POS);
        onEncoderPulse();
        Serial.println(ADAS.target);
    } else {
        /*
            Things to do while descending or on the ground
        */
        //retract ADAS fins

    }
    loopcount++;
}

/// PID SHIT

#include "adas_pid.h"
//constants
float k_d = 0.1;
float k_p = 0.9;
float signal_to_ADAS_ratio = 0.1;  //converts between the signal units to the deployment percentage units


//global variables
int function_index = 1; //keep track of which function we are currently calculating with

//functions
int start_heights[] = {298, 396, 489, 577, 660, 738, 812, 881, 946, 1008, 1066, 1120, 1172, 1220, 1265, 1306, 1345, 1381, 1415, 1437, 1454, 1470, 1486, 1501, 1514, 1527, 1539, 1549, 1559, 1568, 1576, 1584, 1590, 1595, 1600, 1604, 1606, 1608, 1609};

//double velocity_function1[] = {-1.94990816e-40, 4.31360870e-37, -4.03552040e-34, 1.97352573e-31, -4.47454183e-29, -2.79194722e-27, 4.44008811e-24, -8.45546718e-22, -1.70453995e-19, 1.32156649e-16, -3.68749797e-14, 6.40549202e-12, -7.68453997e-10, 6.56546421e-08, -4.00517292e-06, 1.71885641e-04, -5.03485211e-03, 9.61106407e-02, -1.13281432e+00, 9.28356781e+00, 3.54272273e+00};
//double velocity_function2[] = {-6.15047837e-27, 5.70693178e-23, -2.34204648e-19, 5.58915483e-16, -8.57418948e-13, 8.81699003e-10, -6.14071341e-07, 2.85300345e-04, -8.44114066e-02, 1.42146793e+01, -8.21502738e+02};

double velocity_functions[][3] = {
  { -0.000455874984234 ,  0.216170183163 ,  176.710523811 },
  { 1.04868657812e-05 ,  -0.124391243726 ,  238.721409985 },
  { 7.92643542567e-06 ,  -0.121867443237 ,  238.09878419 },
  { -2.9669386665e-05 ,  -0.0770086057444 ,  224.7201655 },
  { -5.85983840182e-07 ,  -0.117759718173 ,  238.93759641 },
  { -9.63650725967e-07 ,  -0.117073698461 ,  238.637791425 },
  { -2.45244048555e-06 ,  -0.114671767 ,  237.668761566 },
  { -4.85400385168e-06 ,  -0.110416808308 ,  235.783785956 },
  { -9.94098238419e-06 ,  -0.100765719385 ,  231.205588252 },
  { -1.76104026014e-05 ,  -0.0852958397462 ,  223.403682762 },
  { -2.62789842423e-05 ,  -0.0668150640905 ,  213.5528557 },
  { -3.59707836192e-05 ,  -0.0450937677849 ,  201.381490807 },
  { -4.71445236453e-05 ,  -0.0189039780727 ,  186.034211748 },
  { -6.05064680359e-05 ,  0.0136992809133 ,  166.145195871 },
  { -7.68529722073e-05 ,  0.0550534303274 ,  139.989260124 },
  { -9.7330441225e-05 ,  0.108575152886 ,  105.015874164 },
  { -0.000124134067604 ,  0.18072143409 ,  56.4662923145 },
  { -0.000160151635125 ,  0.280264534494 ,  -12.3128839095 },
  { -0.000201442981411 ,  0.396962285167 ,  -94.7667852488 },
  { -0.000242138394989 ,  0.513843709934 ,  -178.691181158 },
  { -0.000288541421794 ,  0.64879502734 ,  -276.809459232 },
  { -0.000346447983581 ,  0.819110120969 ,  -402.042870368 },
  { -0.000419664626094 ,  1.03672808983 ,  -563.747025569 },
  { -0.00051360798885 ,  1.3186884767 ,  -775.315703504 },
  { -0.000636158067973 ,  1.68984862901 ,  -1056.344088 },
  { -0.000799086511593 ,  2.18743601704 ,  -1436.25585816 },
  { -0.00102072532098 ,  2.86954097586 ,  -1961.06016126 },
  { -0.00133010049267 ,  3.82836671438 ,  -2703.96735091 },
  { -0.00177503542906 ,  5.21615510456 ,  -3786.12498453 },
  { -0.00243830432254 ,  7.29690176513 ,  -5418.00872656 },
  { -0.0034714283374 ,  10.5546767442 ,  -7986.21589242 },
  { -0.0051716052346 ,  15.9403897623 ,  -12251.3488201 },
  { -0.00817502186666 ,  25.492319445 ,  -19845.9828478 },
  { -0.0140087260584 ,  44.1087432237 ,  -34698.0960943 },
  { -0.026961899123 ,  85.5618903791 ,  -67863.015371 },
  { -0.0622133407046 ,  198.630881534 ,  -158530.21714 },
  { -0.198323557315 ,  635.951816446 ,  -509807.320707 },
  { -1.32577931543 ,  4262.63229352 ,  -3426287.93913 },
  { -53.2846829669 ,  171488.316639 ,  -137976998.018 }};

double calc_velocity(float height){
  function_index = 0;
  while (height > start_heights[function_index] && function_index < sizeof(start_heights)/sizeof(int)){
    function_index++;
  }
  if(function_index == sizeof(start_heights)/sizeof(double)){
    return 0; //retract fins
  }
  int order = sizeof(velocity_functions[function_index])/sizeof(double)-1;

  double function_value = 0;
  int index = 0;
  while (index <= order){
     function_value = function_value + velocity_functions[function_index][index]*pow(height,(order-index));
     index++;
   }
   return function_value;
}


/*
 * returns the new deployment of ADAS
 */
double PID(float my_height, double my_velocity, float prev_signal, float delta_t){ 
  
  float wanted_velocity = calc_velocity(my_height);
  Serial.println("Wanted Vel: ");
  Serial.println(wanted_velocity);
  Serial.println("My velocity");-
  Serial.println(my_velocity);
  
  float cur_signal = (my_velocity-wanted_velocity)*signal_to_ADAS_ratio;//converted to a number between 0 and 1 ish so that its comparable to prev_signal
  Serial.println("Prop signal");
  Serial.println(cur_signal);
  
  float deriv_signal = (cur_signal-prev_signal)/delta_t*signal_to_ADAS_ratio; //cur_sig-prev_sig is approx between 0 and 1, divided by delta t is on the order of 100-1000 so multiply by ///////////NEEED TO FIND TEH FREQUENCY TO MAKE THIS GUUUUUDD
  //don't do integral control for now, not worth it and isn't effective
  Serial.println("Deriv signal");
  Serial.println(deriv_signal);
  float final_signal = (k_p * cur_signal + k_d * deriv_signal);
  
  float new_deployment = prev_signal + final_signal;
  Serial.println("New Depl");
  Serial.println(new_deployment);
  //check that the new deployment is not out of the desired range of 0 to 1
  if(new_deployment > 1){
    new_deployment = 1;
  }
  if(new_deployment < 0){
    new_deployment = 0;
  }
  
  return new_deployment;
}
