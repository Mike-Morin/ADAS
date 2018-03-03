#include "MS5607/IntersemaBaro.h"
#include <CurieTimerOne.h>
#include <SPI.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <math.h>
#include "MPU6050.h"

const int DATABUFFER_LENGTH = 10;

const int IMU_SAMPLE_RATE = 3200; // (hz)
const int IMU_ACCEL_RANGE = 16; // (g)

// declare sensors

IntersemaBaro::BaroPrssure_MS5607B MS5607alt(true);
MPU6050 IMU;
Madgwick filter;
state adas_state;


enum dir {
    EXTEND,
    RETRACT,
    HALT
};

enum flight_state {
    INITIALIZING,
    READY,
    LAUNCHED,
    DESCENDING,
    LANDED
}


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

typedef struct {
    int position;
    int target;
    dir direction;
    flight_state stage;
} state;


// data stuff

dataframe databuffer[DATABUFFER_LENGTH];
int current_index = 0;


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
        current_frame.EX_accel[0],
        current_frame.EX_accel[1],
        current_frame.EX_accel[2],
        current_frame.EX_gyro[0],
        current_frame.EX_gyro[1],
        current_frame.EX_gyro[2]
    );

    filter.update(
        current_frame.EX_accel[0],
        current_frame.EX_accel[1],
        current_frame.EX_accel[2],
        current_frame.EX_gyro[0],
        current_frame.EX_gyro[1],
        current_frame.EX_gyro[2]
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

// PID STUFF

const float k_d = 0.1;
const float k_p = 0.9;
const float signal_to_ADAS_ratio = 0.01;  //converts between the signal units to the deployment percentage units

int function_index = 1; // the fucntion that is currently being used
double prev_signal = 0; // previous signal ouput

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

double PID(float my_height, double my_velocity, float prev_signal, float delta_t){ 
  
  float wanted_velocity = calc_velocity(my_height);
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

// ACTUAL LOOP STUFF

unsigned long prev_micros;

void setup() {
    Wire.begin();


    // initialize sensors
    MS5607alt.init();
    CurieIMU.begin();
    IMU.initialize();
    MPU6050.initialize();


    // set IMU rates
    CurieIMU.setRate(IMU_SAMPLE_RATE);
    MPU6050.setRate(IMU_SAMPLE_RATE);

    // set IMU accelerometer ranges
    CurieIMU.setAccelerometerRange(IMU_ACCEL_RANGE);
    IMU.setFullScaleAccelRange(IMU_ACCEL_RANGE);

    // initialize Madgwick filter
    filter.begin(IMU_SAMPLE_RATE);

    // configure curie interrupts for launc
    CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 9.81*LAUNCH_THRESHOLD_SHOCK*1000);
    CurieIMU.setDetectionThreshold(75);

    // timing stuff
    micros_per_reading = 1000000/IMU_SAMPLE_RATE;
    prev_micros = micros();

    CurieTimerOne.start(WATCHDOG_LIMIT, &onWatchdogTimeout);

    // setup pins
    pinMode(hb_in1_pin, OUTPUT);
    pinMode(hb_in2_pin, OUTPUT);
    pinMode(hb_en_pin, OUTPUT);
    pinMode(encoder_pin, OUTPUT);
    pinMode(limitswitch_pin, OUTPUT);


    state = READY;
}

int loop_counter = 0;

void loop() {
    CurieTimerOne.restart(WATCHDOG_LIMIT);

    if (state == READY) {
        if (loop_counter %2 == 0) { // gather data half as often (change number to 1/x the amount of data you want to collect while on the ground)
            get_data();
        }
    }  else if (state == LAUNCHED || state == DESCENDING) {
        get_data();
        if (state == LAUNCHED) { // only execute pid stuff when launched but not in air
            float current_time;
            float prev_time;
            if (current_index == 0) {
                current_time = databuffer[DATABUFFER_LENGTH-1].ts;
                prev_time = databuffer[DATABUFFER_LENGTH-2].ts;
            } else {
                cur_time = databuffer[current_index-1];
                prev_time = databuffer[current_time-2];
            }

            float time_diff = cur_time - prev_time;

            double new_target = PID(databuffer.position[])
        }

    } else if (state == LANDED) {
        if (loop_counter %2 == 0) { // gather data half as often (change number to 1/x the amount of data you want to collect while on the ground)
            get_data();
        }
    }


    write_data();
    loop_counter++;
}

