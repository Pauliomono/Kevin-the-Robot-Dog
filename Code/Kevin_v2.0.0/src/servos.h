#ifndef SERVOS_H
#define SERVOS_H

#include <Arduino.h>
#include <Servo.h>

extern Servo hip1;
extern Servo shoulder1;
extern Servo knee1; 
extern Servo hip2;
extern Servo shoulder2;
extern Servo knee2; 
extern Servo hip3;
extern Servo shoulder3;
extern Servo knee3; 
extern Servo hip4;
extern Servo shoulder4;
extern Servo knee4;

//leg angle structure
struct angles{
  float hip;
  float shoulder;
  float knee;
};

extern struct angles leg1;
extern struct angles leg2;
extern struct angles leg3;
extern struct angles leg4;

void write_servos();

#endif