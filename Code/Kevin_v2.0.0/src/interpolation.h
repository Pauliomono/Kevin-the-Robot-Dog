#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <Arduino.h>
#include <MatrixMath.h>
#include <configuration.h>

extern int interpolation_interval;

struct outputs{
    float pos;
    float vel;
};

float interpolate2(float thetaf, float theta, int t0, int tf, int t);
float interpolate1(float x_f, float x_0, float xdot_f, float xddot_f, float t0, float tf, float t);
struct outputs interpolate3(float x_f, float x_c, float xdot_f, float xdot_c, float t0, float tf, float t);

#endif