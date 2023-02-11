#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <servos.h>
#include <interpolation.h>

extern int state;
extern int state_time;

// foot position vars
extern float L;
extern float l_pitch;
extern float l_roll;

// leg position control vars
extern float x_dist;
extern float z_dist;
extern double steer1;
extern float steer2;
extern double pitch1;
extern double pitch_angle1;
extern float pitch2;
extern double pitch_angle2;
extern double roll1;
extern double roll_angle1;
extern float roll2;
extern double roll_angle2;
extern double yaw1;
extern float yaw2;
extern float step_height;
extern float leg_height;

struct points
{
  float pos;
  float vel;
  float accel;
  float pos_f;
  float vel_f;
  float accel_f;
};

struct xyz
{
  float x;
  float y;
  float z;
};

struct r
{
  struct xyz r1;
  struct xyz r2;
  struct xyz r3;
  struct xyz r4;
};

extern struct points x1;
extern struct points yy1;
extern struct points z1;
extern struct points x2;
extern struct points yy2;
extern struct points z2;
extern struct points x3;
extern struct points yy3;
extern struct points z3;
extern struct points x4;
extern struct points yy4;
extern struct points z4;

extern struct r shoulders;
extern float L_body;
extern float L_shoulder;
extern float L_COM;
extern double roll_commanded;
extern double pitch_commanded;
extern double yaw_commanded;

struct angles IK_final(float rx, float ry, float rz);
struct angles IK_interpolate(struct angles leg_final, struct angles leg, float t0, float tf, float t);
void IK_interpolate2(float t0, float tf, float t);
void get_xyz();
void offset();
struct r attitude_model(double roll, double pitch, double yaw);

#endif