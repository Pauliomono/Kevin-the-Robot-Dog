#ifndef INTERPRET_INPUT2_H
#define INTERPRET_INPUT2H

#include <Arduino.h>
#include <configuration.h>

extern bool data_recieved;
extern String datapacket;
extern int mode;
extern float x_dist;
extern double steer1;
extern float steer2;
extern int motion_limit;
extern int state_time;

extern int xxx;
extern int yyy;
extern int packet;

void interpret_input2(int packet);

#endif