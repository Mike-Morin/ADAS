void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  


}

//cosntants
float k_d = 0.1;
float k_p = 0.9;
float signal_to_ADAS_depl = 90;
float delta_t = 0.01;


//global variables
int function_index = 1; //keep track of which function we are currently calculating with
double prev_signal = 0;  //the cur_signal from the previous loop

//functions ///MIGHT NEED TO ADD ZERO IN THE BEGINING
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

void loop() {
  Serial.println("Start");
  Serial.println(calc_velocity(200));
  Serial.println(calc_velocity(400));
  Serial.println(calc_velocity(600));
  Serial.println(calc_velocity(800));
  Serial.println(calc_velocity(1000));
  Serial.println(calc_velocity(1200));
  Serial.println(calc_velocity(1400));
  Serial.println(calc_velocity(1600));
  /*
  // put your main code here, to run repeatedly:
  
  float my_height = 0; //read height from the sensors
  float my_velocity = 0; //calculate the current velocity
  
  float wanted_velocity = calc_velocity(my_height);
  float cur_signal = wanted_velocity-my_velocity;

  float deriv_signal = (cur_signal-prev_signal)/delta_t; ////////////////////I dont know how to do time stamp things/////////////////

  //don't do integral control for now, not worth it and isn't effective

  float final_signal = (k_p * cur_signal + k_d * deriv_signal)*signal_to_ADAS_depl;

  //update variables
  prev_signal = cur_signal;
  */
}




