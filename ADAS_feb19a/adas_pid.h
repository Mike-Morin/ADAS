#ifndef _ADAS_PID_INCLUDED
#define _ADAS_PID_INCLUDED

float k_d;
float k_p;
double calc_velocity(float height);
double PID(float my_height, double my_velocity, float prev_signal, float delta_t);

#endif
