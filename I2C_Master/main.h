#ifndef MAIN_H
#define MAIN_H

#include "mbed.h"

class Servo {
public:
    void torque (uint8_t ID, int data);
    void rotate (uint8_t ID, int data);   

};

class Joint {
public:
    uint8_t id;
    float phi;
    float theta;
    float omg;
    float tau;
    float N;
    float N_off;
};
    
#endif