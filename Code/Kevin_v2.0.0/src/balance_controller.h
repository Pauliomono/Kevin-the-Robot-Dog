#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>
#include <kinematics.h>

extern double pitch1;
extern double roll1;
extern float pitch2;
extern double pitch_angle1;
extern double pitch_angle2;
extern double roll_angle1;
extern double roll_angle2;

extern float leg_height;

void balance();

#endif