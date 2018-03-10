#ifndef _ADAS_PID_INCLUDED
#define _ADAS_PID_INCLUDED

double calc_velocity(float height);
float PID(float my_height, float my_velocity, float prev_signal, float delta_t);

#endif
