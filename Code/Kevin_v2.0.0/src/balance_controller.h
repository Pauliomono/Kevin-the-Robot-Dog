#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <Arduino.h>
#include <configuration.h>
#include <kinematics.h>
#include <PID_v1.h>

extern double pitch1;
extern double roll1;
extern float pitch2;
extern double pitch_angle1;
extern double pitch_angle2;
extern double roll_angle1;
extern double roll_angle2;

extern float leg_height;

extern int mode;
extern int state;

extern PID pitchPID;
extern PID rollPID;
extern PID yawPID;

// input_shape globals
extern double frequency;
extern double amplitude;
extern int root_time;
extern int root_time_counter; // global?
extern double low_trigger;
extern double high_trigger;
extern double peak;
extern double trough;
extern int peak_time;
extern int trough_time;
extern int root_time_1; // global?
extern int root_time_2;
extern bool trigger_state; // global?
extern int step; // global
extern int roots_delta;
extern double mpeak;
extern double mtrough;
extern int t_sin_init;
extern int phase_time;

void balance();
int input_shape_setup(double target_angle, double feedback_angle, int n_sample_count);
double input_shaper(double input, double f, double A);

#endif