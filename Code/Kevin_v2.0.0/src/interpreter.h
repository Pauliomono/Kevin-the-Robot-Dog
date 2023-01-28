#ifndef INTERPRET_INPUT2_H
#define INTERPRET_INPUT2H

#include <Arduino.h>
#include <configuration.h>
#include <PID_v1.h>

extern bool data_recieved;
extern String datapacket;
extern int mode;
extern float x_dist;
extern double steer1;
extern float steer2;
extern int motion_limit;
extern int state_time;
extern int n_interps;
extern int interpolation_interval;

extern int xxx;
extern int yyy;
extern int packet;

extern int tune_precision;
extern int xxx_old;
extern int yyy_old;
extern float x_tune;
extern float y_tune;
extern int tune_mode;

extern float kpr;
extern float kir;
extern float kdr;

extern float kpp;
extern float kip;
extern float kdp;

extern PID pitchPID;
extern PID rollPID;

void interpret_input2(int packet);
void interpret_input_string();

#endif