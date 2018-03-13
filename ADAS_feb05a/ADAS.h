#ifndef ADAS_h
#define ADAS_h
# include "Arduino.h"
class ADAS_state {
  bool launched, inFatalError;
  unsigned long launch_time;
  int dir, pulse_count, error, desired_pos;
  int hbridge1, hbridge2;
  public:
    void setLaunched();
    bool isLaunched();
    unsigned long getLaunchTime();
    void setDir(int dir);
    int getDir();
    int getPulseCount();
    void zeroPulseCount(); // zeros pulse count
    void setError(int err);
    int getError();
    void setPos(int pos);
    int getPos();
    void onPulse();
    void move();
    void move(int direction);
    void update();
};

#endif
