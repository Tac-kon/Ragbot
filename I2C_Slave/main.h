#ifndef MAIN_H
#define MAIN_H

#include "mbed.h"

class Servo {
public:
    void torque (uint8_t ID, int data);
    void rotate (uint8_t ID, int data); 
    double readangle (uint8_t ID); 
    double readomega (uint8_t ID);
private:
    void rt_pkt (uint8_t ID, uint8_t Adr);
};

class Joint {
public:
    uint8_t id;
    double  phi;
    double pp;
    double v_rot;
    double gp;
    double tau;
    double N;
    double N_off;
};
  
#endif