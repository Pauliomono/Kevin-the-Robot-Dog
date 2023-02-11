#include <interpolation.h>

// linear interpolation function - constant velocity
float interpolate2(float thetaf, float theta, int t0, int tf, int t)
{
  tf = tf - t0;
  t = t - t0;
  

  int n_intervals = (tf - t) / interpolation_interval;
  if (n_intervals == 0)
  {
    n_intervals = 1;
  }
  float dtheta = (float)(thetaf - theta) / n_intervals;
  theta = dtheta + theta;
  return (theta);
}

// cubic spline interpolation function - way better
float interpolate1(float x_f, float x_c, float xdot_f, float xddot_f, float t0, float tf, float t)
{
  float x;
  // mtx_type A[6][6];
  tf = (tf - t);
  float tc = 0;

  float t_next = (tc + interpolation_interval);
 
  // Matrix for system of equations
  mtx_type A[6][6] = {
      {pow(tf, 3) / 6, pow(tf, 2) / 2, tf, 1, 0, 0},
      {0, 0, 0, 1, 0, 0},
      {pow(tf, 2) / 2, tf, 1, 0, 0, 0},
      {0, 0, 1, 0, -1, 0},
      {tf, 1, 0, 0, 0, 0},
      {0, 1, 0, 0, 0, -1}};

  // Matrix for solutions to system
  mtx_type C[6] = {x_f, x_c, xdot_f, 0, xddot_f, 0};

  // Matrix of unkown a-values
  mtx_type X[6];

// solve system
#ifdef PRINT_INTERPOLATION
  Matrix.Print((mtx_type *)A, 6, 6, "A");
  Serial.print("\t");
  Matrix.Print((mtx_type *)C, 6, 1, "C");
  Serial.print("\t");
#endif
  if (Matrix.Invert((mtx_type *)A, 6)) {

    //Matrix.Invert((mtx_type *)A, 6);

    Matrix.Multiply((mtx_type *)A, C, 6, 6, 1, X);

#ifdef PRINT_INTERPOLATION
    Matrix.Print((mtx_type *)A, 6, 6, "A inverted");
    Serial.print("\t");
    Matrix.Print((mtx_type *)X, 6, 1, "X");
    Serial.print("\t");
#endif
    float a0 = X[0];
    float a1 = X[1];
    float a2 = X[2];
    float a3 = X[3];
    // float xdot_0 = X[4];
    // float xddot_0 = X[5];

    // compute next coordinate
    x = (float)a0 * pow(t_next, 3)/6 + a1 * pow(t_next, 2)/2 + a2 * t_next + a3;

#ifdef PRINT_INTERPOLATION
    Serial.print(a0);
    Serial.print("\t");
    Serial.print(a1);
    Serial.print("\t");
    Serial.print(a2);
    Serial.print("\t");
    Serial.print(a3);
    Serial.print("\t");
    Serial.print(x);
    Serial.print("\t");
#endif
  }
  else{
    x = x_c;


  }

  return x;
}

// inverted matrix
/*
float o1 = 3 * pow(tf, 2) / pow(t0 - tf, 3);
float o2 = 6 * tf / pow(t0 - tf, 3);
float o3 = pow(tf, 3) / pow(t0 - tf, 3);
float o4 = 6 / pow(t0 - tf, 3);
float o5 = 6 / pow(t0 - tf, 2);
float o6 = -3 / (t0 - tf);

float A[6][6] = {
  {-o4, o4, -o5, 0, o6, 0},
  {o2, -o2, (6 * tf) / pow(t0 - tf, 2), 0, 1 + (3 * tf) / (t0 - tf), 0},
  {-o1, o1, 1 - 3 * pow(tf, 2) / pow(t0 - tf, 2), 0, -tf - 3 * pow(tf, 2) / (2 * (t0 - tf)), 0},
  {o3 + 1, -o3, pow(tf, 3) / pow(t0 - tf, 2) - tf, 0, (t0 * pow(tf, 2)) / (2 * (t0 - tf)), 0},
  {o6, 3 / (t0 - tf), -2, -1, .5 * (tf - t0), 0},
  {-o5, o5, -6 / (t0 - tf), 0, -2, -1}
};
*/

struct outputs interpolate3(float x_f, float x_c, float xdot_f, float xdot_c, float t0, float tf, float t)
{
  struct outputs x;
  
  // mtx_type A[6][6];
  tf = (tf - t);
  float tc = 0;

  float t_next = (tc + interpolation_interval);
  

  // Matrix for system of equations
  mtx_type A[4][4] = {
      {pow(tf, 3) / 6, pow(tf, 2) / 2, tf, 1},
      {0, 0, 0, 1},
      {pow(tf, 2) / 2, tf, 1, 0},
      {0, 0, 1, 0}};

  // Matrix for solutions to system
  mtx_type C[4] = {x_f, x_c, xdot_f, xdot_c};

  // Matrix of unkown a-values
  mtx_type X[4];

// solve system
#ifdef PRINT_INTERPOLATION
  Matrix.Print((mtx_type *)A, 4, 4, "A");
  Serial.print("\t");
  Matrix.Print((mtx_type *)C, 4, 1, "C");
  Serial.print("\t");
#endif
  if (Matrix.Invert((mtx_type *)A, 4)) {

    //Matrix.Invert((mtx_type *)A, 4);

    Matrix.Multiply((mtx_type *)A, C, 4, 4, 1, X);

#ifdef PRINT_INTERPOLATION
    Matrix.Print((mtx_type *)A, 4, 4, "A inverted");
    Serial.print("\t");
    Matrix.Print((mtx_type *)X, 4, 1, "X");
    Serial.print("\t");
#endif
    float a0 = X[0];
    float a1 = X[1];
    float a2 = X[2];
    float a3 = X[3];
    // float xdot_0 = X[4];
    // float xddot_0 = X[5];

    // compute next coordinate
    x.pos = (float)a0 * pow(t_next, 3)/6 + a1 * pow(t_next, 2)/2 + a2 * t_next + a3;
    x.vel = (float)a0 * pow(t_next, 2)/2 + a1 *t_next + a2;

#ifdef PRINT_INTERPOLATION
    Serial.print(a0);
    Serial.print("\t");
    Serial.print(a1);
    Serial.print("\t");
    Serial.print(a2);
    Serial.print("\t");
    Serial.print(a3);
    Serial.print("\t");
    Serial.print(x.pos);
    Serial.print("\t");
    Serial.print(x.vel);
    Serial.print("\t");
#endif
  }
  else{
    x.pos = x_c;
    x.vel = xdot_c;


  }

  return x;
}