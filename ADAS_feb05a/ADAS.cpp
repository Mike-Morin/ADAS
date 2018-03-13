#include "Arduino.h"
#include "ADAS.h"

const int ERROR = 2;
ADAS_state::ADAS_state() {};
ADAS_state::ADAS_state(int hbridge1_pin, int hbridge2_pin) {
  hbridge1 = hbridge1_pin;
  hbridge2 = hbridge2_pin;
  pinMode(hbridge1, OUTPUT);
  pinMode(hbridge2, OUTPUT);
}

void ADAS_state::setLaunched(){
  launched = true;
  launch_time = millis(); // probably should be from the external clock
};

bool ADAS_state::isLaunched() {
  return launched;
}

unsigned long ADAS_state::getLaunchTime() {
  return launch_time;
}

void ADAS_state::setDir(int dir) {
  if (-1 < dir && dir < 1) {
    dir = dir;
  }
}

int ADAS_state::getDir() {
  return dir;
}

int ADAS_state::getPulseCount() {
  return pulse_count;
}

void ADAS_state::zeroPulseCount() {
  pulse_count = 0;
}

int ADAS_state::getError() {
  return error;
}

void ADAS_state::setPos(int pos) {
  if (0 < pos && pos <=MAX_POS) {
    desired_pos = pos;
  }
}

int ADAS_state::getPos() {
  return desired_pos;
}

void ADAS_state::onPulse() {
  pulse_count+=dir;
  if (pulse_count >= (desired_pos-ERROR) && pulse_count <= desired_pos+ERROR) {
    dir = 0;
    digitalWrite(hbridge1, HIGH);
    digitalWrite(hbridge2, HIGH);
  }
  if (pulse_count >= MAX_POS) {
    dir = 0;
    desired_pos = MAX_POS;
    digitalWrite(hbridge1, HIGH);
    digitalWrite(hbridge1, HIGH);
  }
}

void ADAS_state::move() {
  move(dir);
}

void ADAS_state::move(int direction) {
  if (direction == 1) {
    digitalWrite(hbridge1, HIGH);
    digitalWrite(hbridge2, LOW);
  } else if (direction == -1) {
    digitalWrite(hbridge1, LOW);
    digitalWrite(hbridge2, HIGH);
  } else if (direction == 0) {
    digitalWrite(hbridge1, HIGH);
    digitalWrite(hbridge2, HIGH);
  }
}

void ADAS_state::update() {
  move();
  if (pulse_count <= (desired_pos - ERROR)) {
    dir=1;
  } else if (pulse_count >= (desired_pos + ERROR)) {
    dir=-1;
  }
}

void ADAS_state::setError(int err) {
  error = err;
}
