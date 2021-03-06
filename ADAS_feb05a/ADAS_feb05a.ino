#include "MS5607/IntersemaBaro.h"

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <CurieIMU.h>
#include "MPU6050.h"
#include "MadgwickAHRS.h"
#include "ADAS.h"

#define DEBUG 0 // not in debug mode

const int IMU_UPDATE = 50;

/* Pin defs */
const byte hbridgeIN1pin = 2; //h-bridge board pins 2 & 3
const byte beeperpin = 3;
const byte limitswitchpin = 4;
const byte hbridgeENpin = 5;
const byte hbridgeIN2pin = 6;//h-bridge board pins  1 & 4
const byte encoderpinA = 7;
const byte sdpin = 10; // Also known as SS.


/* Pin defs in the SPI library
   SS -- pin 10
   MOSI -- pin 11
   MISO -- pin 12
   SCK  -- pin 13
*/



/* ADAS variables */

ADAS_state ADAS(hbridgeIN1pin, hbridgeIN2pin);

/* Data variables */
unsigned long tsbuf[10];
float ADASdatabuf[12][10];

Intersema::BaroPressure_MS5607B MS5607alt(true);
MPU6050 IMU;
Madgwick filter;

File ADASdatafile;

void beep(int code) {
  /*
    Plays informational beeps, codes are integers, negative codes are errors,
    positive codes are purely informational
    error code -99 is an extreme error and is indicated by a constant tone
  */

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


float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

/*
 * Returns the acceleration in m/s^2
 */

float convertRawAcceleration(int aRaw) {
  // since we are using 16G range
  // -16g maps to a raw value of -32768
  // +16g maps to a raw value of 32767

  float a = (aRaw * 16) / (32768.0);
  return a;
}

unsigned long get_time() {
  unsigned long ts = 0;
  Wire.requestFrom(8, 4);    // request 2 bytes from slave device #8
  while (Wire.available()){
    ts = ts << 8;
    ts |= Wire.read();
  }
  return ts;
}

boolean first_time = true;
float g_feel;
float g_conv;
float initial_height;//in cm

void getData(int i) {
  /*
    Polls all the sensors and puts the data in the ADASdatabuf.
    The acclerometer gets polled 10 times for every altimeter
    reading because the altimeter is very slow (fix!?).
  */

    tsbuf[i] = get_time();

    int16_t ax, ay, az, gx, gy, gz;
    IMU.getMotion6(&ax,
                   &ay,
                   &az,
                   &gx,
                   &gy,
                   &gz
    );
    // convert everything to rational units
    ADASdatabuf[0][i] = convertRawAcceleration(ax);
    ADASdatabuf[1][i] = convertRawAcceleration(ay);
    ADASdatabuf[2][i] = convertRawAcceleration(az);
    ADASdatabuf[3][i] = convertRawGyro(gx);
    ADASdatabuf[4][i] = convertRawGyro(gy);
    ADASdatabuf[5][i] = convertRawGyro(gz);

    float g = 9.81; //in m/s^2
    filter.updateIMU(
      ADASdatabuf[3][i],
      ADASdatabuf[4][i],
      ADASdatabuf[5][i],
      ADASdatabuf[0][i],
      ADASdatabuf[1][i],
      ADASdatabuf[2][i]
    );

    ADASdatabuf[6][i] = filter.getPitch();
    //ADASdatabuf[7][i] = filter.getRoll();
    //ADASdatabuf[8][i] = filter.getYaw();

    if(first_time){//get initial height and g value readings
      initial_height = MS5607alt.getHeightCentiMeters();
      g_feel = sqrt(pow(ADASdatabuf[0][i],2)+pow(ADASdatabuf[1][i],2)+pow(ADASdatabuf[2][i],2));
      g_conv = g/g_feel;
      first_time = false;
      initialDataWrite(g_conv, initial_height);
    }


    //PID needs the vertical velocity and height
    //9 is vertical velocity
    //10 is vertical position

      float prev_vert_velocity = 0;
      if(i != 0){
        prev_vert_velocity = ADASdatabuf[10][i-1];
      } else {
        prev_vert_velocity = ADASdatabuf[10][9];
      }

      float vertical_acc = g_conv*ADASdatabuf[0][i]-g;
      float prev_t = 0;
      float prev_h = 0;
      if(i != 0){
        prev_t = tsbuf[i-1];
        prev_h = ADASdatabuf[11][i-1];
      } else {
        prev_t = tsbuf[9];
        prev_h = ADASdatabuf[11][9];
      }
      float delta_t = (tsbuf[i] - prev_t)/1000; //in seconds
      float new_vert_height = prev_h + delta_t*prev_vert_velocity;
      float new_vert_velocity = prev_vert_velocity + vertical_acc * delta_t;

      ADASdatabuf[10][i] = new_vert_velocity;
      ADASdatabuf[11][i] = new_vert_height;

      //need to do this every 75ms ish, this is about every 10 iterations
      if(i == 9 && !ADAS.isLaunched()){//reset the integrator velocity
        ADASdatabuf[10][i] = 0;
      }
#if DEBUG
      Serial.print(ADASdatabuf[10][i]);
      Serial.print("       ");
      Serial.println(ADASdatabuf[11][i]);
#endif

    if (i == 0) { // The altimeter is polled once, the integrated height is reset, and the mpu6050 fifo is cleared.
      IMU.resetFIFO();
      ADASdatabuf[9][i] = MS5607alt.getHeightCentiMeters()-initial_height; //in cm
      ADASdatabuf[11][i] = ADASdatabuf[9][i]/100; //in meters
    } else {
      ADASdatabuf[9][i] = ADASdatabuf[9][i - 1];
    }

}


void initialDataWrite(float g, float height){

  ADASdatafile = open("ADASdata.txt", FILE_WRITE);
  if (ADASdatafile) {
    ADASdatafile.print("g: ");
    ADASdatafile.print(g);
    ADASdatafile.print("\n");

    ADASdatafile.print("Initial height (cm): ");
    ADASdatafile.print(height);
    ADASdatafile.print("\n");
  } else {
    // if the file didn't open, print an error:
    // NOTE: this doesnt do much in the air
#if DEBUG
    Serial.println("error opening test.txt");
#endif
    ADAS.setError(-9);
    beep(-9);
  }
  close(ADASdatafile);
}
void writeData() {
  /*
    Writes all the sensor data to SD card
  */

  /* Interrupts must be disabled or the SD card
     will be corrupted upon write.
  */

  ADASdatafile = open("ADASdata.txt", FILE_WRITE);

  if (ADASdatafile) {
    for (int j = 0; j < 10; j++) { //10 blocks of
      ADASdatafile.print(tsbuf[j]);
      ADASdatafile.print(",");
       for (int i = 0; i < 12; i++) { //9 sensor outputs + 3 for phun
        ADASdatafile.print(ADASdatabuf[i][j]);
        ADASdatafile.print(",");
      }
      ADASdatafile.print(ADAS.isLaunched());
      ADASdatafile.print(",");
      ADASdatafile.print(ADAS.getPulseCount());
      ADASdatafile.print(",");
      ADASdatafile.print(ADAS.getPos());
      ADASdatafile.print(",");
      ADASdatafile.print(ADAS.getDir());
      ADASdatafile.print("\n");
    }
  } else {
    // if the file didn't open, print an error:
    // NOTE: this doesnt do much in the air
#if DEBUG
    Serial.println("error opening test.txt");
#endif
    ADAS.setError(-9);
    beep(-9);
  }
  close(ADASdatafile);
}
void onEncoderPulse() {
  ADAS.onPulse();
}
void onLaunch() {
  ADAS.setLaunched();
}

void AttachInterrupts() {
  /*
    All interrupts that should be detached for sd card reading/writting
  */
  attachInterrupt(digitalPinToInterrupt(encoderpinA), onEncoderPulse, RISING); //Catch interrupts from the encoder.
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
  DetachInterrupts();
  return SD.open(filename, mode);
}

void close(File file) {
  /*
    reattaches interrupts after closing the file to avoid corruption of the file in question
  */
  file.close();
  AttachInterrupts();
}

void setup() {

  Wire.begin();
  Serial.begin(9600);

  /* For the altimeter */
  MS5607alt.init();

  //CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 75);
  /* For the built-in Curie IMU */
  CurieIMU.begin();
  CurieIMU.setAccelerometerRange(16);
  CurieIMU.setGyroRate(25);//was 3200
  CurieIMU.setAccelerometerRate(12.5);//was 12.5


  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 5031.25);
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 75);
  CurieIMU.attachInterrupt(onLaunch);


  IMU.initialize();
  IMU.setFullScaleAccelRange(3);//max of 16g's
  IMU.setRate(IMU_UPDATE);
  IMU.getIntDataReadyEnabled();

  filter.begin(IMU_UPDATE);

  while (!SD.begin(sdpin)) { //Stop everything if we cant see the SD card!
#if DEBUG
    Serial.println("Card failed or not present.");
#endif
    beep(-1);
  }
#if DEBUG
  Serial.println("Card OK");
#endif
  beep(1);

  delay(500);

  /* ADAS control stuff */
  pinMode(hbridgeIN1pin, OUTPUT); //hbridge IN1
  pinMode(hbridgeIN2pin, OUTPUT); //hbridge IN2
  pinMode(hbridgeENpin, OUTPUT); //hbridge EN pin for pwm
  pinMode(encoderpinA, INPUT); //encoder A (or B... either works).
  pinMode(beeperpin, OUTPUT);
  digitalWrite(hbridgeENpin, HIGH);  //SUPER IMPORTANT.

  AttachInterrupts();
}

unsigned int loopcount = 0;
float height_diff = 1;

void loop() {
  loopcount++;
  getData(loopcount % 10);
  if (loopcount%10 == 9) { // dump data on the last round, make zafar happy
    writeData();
  }

  //if(!ADAS.isLaunched()){////FOR TESTING
    //digitalWrite(beeperpin, HIGH);
  //}
  if (ADAS.isLaunched()) {
    digitalWrite(beeperpin, LOW);
    float prev_deployment = (float)(ADAS.getPos())/ADAS.MAX_POS;  //the ratio of the previous deployment to full deployment
    float cur_time = tsbuf[loopcount%10]/1000; //in seconds
    float velocity = ADASdatabuf[10][loopcount%10];
    float height = ADASdatabuf[11][loopcount%10];

    float prev_time;
    if(loopcount%10 == 0){
      prev_time = tsbuf[9];
    } else{
      prev_time = tsbuf[loopcount%10-1];
    }

    float time_diff = cur_time - prev_time;
    float new_ADAS_deployment = PID(height, velocity, prev_deployment, time_diff);

    ADAS.setPos((int) (new_ADAS_deployment*ADAS.MAX_POS));
  }
  ADAS.update();
}



///////////////PID STUFF/////////////////////////
//cosntants
float k_d = 0.1;
float k_p = 0.9;
float signal_to_ADAS_ratio = 0.1;


//global variables
int function_index = 1; //keep track of which function we are currently calculating with
double prev_signal = 0;  //the cur_signal from the previous loop


int start_heights[] = {298, 396, 489, 577, 660, 738, 812, 881, 946, 1008, 1066, 1120, 1172, 1220, 1265, 1306, 1345, 1381, 1415, 1437, 1454, 1470, 1486, 1501, 1514, 1527, 1539, 1549, 1559, 1568, 1576, 1584, 1590, 1595, 1600, 1604, 1606, 1608, 1609};

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
  if(height < start_heights[0]){
    return 0;
  }
  while (height > start_heights[function_index] && function_index < sizeof(start_heights)/sizeof(int)){
    function_index++;
  }
  if(function_index >= sizeof(start_heights)/sizeof(double)){
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

float PID(float my_height, float my_velocity, float prev_signal, float delta_t){

  float wanted_velocity = (float) (calc_velocity(my_height));

  if(wanted_velocity == 0){
    return 0;
  }

  float cur_signal = (my_velocity-wanted_velocity)*signal_to_ADAS_ratio;//converted to a number between 0 and 1 ish so that its comparable to prev_signal
  float deriv_signal = (cur_signal-prev_signal)/delta_t*signal_to_ADAS_ratio; //cur_sig-prev_sig is approx between 0 and 1, divided by delta t is on the order of 100-1000 so multiply by ///////////NEEED TO FIND TEH FREQUENCY TO MAKE THIS GUUUUUDD
  //don't do integral control for now, not worth it and isn't effective

  float final_signal = (k_p * cur_signal + k_d * deriv_signal);

  float new_deployment = prev_signal + final_signal;

  //check that the new deployment is not out of the desired range of 0 to 1
  if(new_deployment > 1){
    new_deployment = 1;
  }
  if(new_deployment < 0){
    new_deployment = 0;
  }

  return new_deployment;
}
