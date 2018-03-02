#include <math.h>

const int MAX_POS = 220; // number of pulses at max extent
const int SLOW_PWM = 75; // (#/255) pwm of slow signal
const int FAST_PWM = 255; // (#/255) pwm of fast movement signal
const int CREEPUP_THRESHOLD = 10; // (steps) number of steps at creepup threshold
const int HYSTERESIS = 2; // (steps) amount of under or over shoot allowed

// pin definitions
const int pin_hbridgeIN1 = 1;
const int pin_hbridgeIN2 = 2;
const int pin_hbridgeEN = 3; // should be pwm pin
const int pin_limit = 4;
const int pin_encoder = 5;

// globals

enum dir {
    backwards,
    stop,
    forewards
}

enum mode {
    slow,
    fast
}

volatile int current_pos;
volatile int target_pos;
volatile dir current_dir;
volatile mode speed;

bool zeroed = false; // if the motor has been zeroed

void setup() {
    Serial.begin(115200);
    pinMode(pin_hbridgeIN1, OUTPUT);
    pinMode(pin_hbridgeIN2, OUTPUT);
    pinMode(pin_hbridgeEN, OUTPUT);
    pinMode(pin_limit, INPUT);
    pinMode(pin_encoder, INPUT);
}

// interrupts
void onEncoderPulse() {
    // change the current_position
    switch (current_dir) {
        case forwards:
            current_pos++;
            break;
        case backwards:
            current_pos--;
            break;
    }
    // check if it is within limits
    if ((MAX_POS+HYSTERESIS) > current_pos > (MAX_POS-HYSTERESIS) && current_dir == forwards) {
        setMotorDirection(stop);
        return;
    }
    if ((0+HYSTERESIS) > current_pos > (0-HYSTERESIS) && current_dir == backwards) {
        setMotorDirection(stop);
        return;
    }
    // set creepup stuff
    if (speed == fast && abs(current_pos - target_pos) > CREEPUP_THRESHOLD) {
        setMode(slow);
    }
    if (speed == slow && abs(current_pos - target_pos) < CREEPUP_THRESHOLD) {
        setMode(fast);
    }

    if (current_dir == backwards && (target_pos+HYSTERESIS) > current_pos) {
        setMotorDirection(forwards);
    }
    if (current_dir == forwards && current_pos > (target_pos-HYSTERESIS)) {
        setMotorDirection(backwards);
    }
}

void serialEvent() {
    while (Serial.available()) {
        int proposed_val = Serial.parseInt();
        if (MAX_POS >= proposed_val >= 0) {
            target_pos = proposed_val;
        } 
    }
}

void onLimitReached() {
    setMotorDirection(stop);
    current_pos = 0;
}

// other functions

void zero() {
    target_pos = 0;
    setMotorDirection(backwards);
}



void setMode(mode movement_mode) {
    if (movement_mode != speed) {
        speed = movement_mode;
        if (movement_mode == slow){
            analogWrite(pin_hbridgeEN, SLOW_PWM);
        } else {
            analogWrite
        }
    }
}

void setMotorDirection(dir direction) {
    switch (direction) {
        case stop:
            
            break;
        case forwards:
            break;
        case backwards:
            break;

    }
}
