#include <Arduino.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

#include "interpolation.h"
#include <remote_control.h>
#include "IMU.h"
#include <servos.h>
#include <kinematics.h>
#include <balance_controller.h>
#include <debugging.h>
#include <configuration.h>

// rc globals
int xxx;
int yyy;
int packet;
int packet2;
int t_old_rc;

// debug tools
int debug_time;
int debug_point = 1;
int n_debug_points = 3;

// code control vars
int state_time = 100;
int interpolation_interval;
int n_interps = 20;
int state = 1;
int mode = 0;
int n_states = 6;

// timing vars
float t0 = 0;
float tf = 1000;
float t = 0;
int t_old;
int t_old_state;

// SoftwareSerial Remote (21, 20);
String datapacket;
bool data_recieved = 0;

// button stuff

// foot position vars
float L = 124;
float l_pitch = 156;
float l_roll = 76;
struct points x1;
struct points yy1;
struct points z1;
struct points x2;
struct points yy2;
struct points z2;
struct points x3;
struct points yy3;
struct points z3;
struct points x4;
struct points yy4;
struct points z4;

// leg position control vars
float x_dist = 0;
float z_dist = 0;
double steer1 = 0;
float steer2 = 0;
double pitch1 = 0;
double pitch_angle1 = 0;
float pitch2 = 0;
double pitch_angle2 = 0;
double roll1 = 0;
double roll_angle1 = 0;
float roll2 = 0;
double roll_angle2 = 0;
double yaw1;
float yaw2;
float step_height = 20;
float leg_height = 180;
int control_increment = 20;
int motion_limit = 30;

double yaw;
double pitch;
double roll;

double yaw_desired = 0;
double pitch_desired = -2; // 1
double roll_desired = 0;

#ifdef PID_BALANCE_ENABLE
// PID setup
float kup = 4.5;
float tup = 1 * .01613;
float kpp = .05; //.45*kup;
float kip = 0.0; //.54*kup/tup;
float kdp = 0;   //.54*kup*tup;
PID pitchPID(&pitch, &pitch_angle1, &pitch_desired, kpp, kip, kdp, DIRECT);
float kpr = .05;  //.6;
float kir = 0.01; //.01;
float kdr = 0.01; //.04;
PID rollPID(&roll, &roll_angle1, &roll_desired, kpr, kir, kdr, DIRECT);
float kpy = 0.0;
float kiy = 0.0;
float kdy = 0.0;
PID yawPID(&yaw, &yaw1, &yaw_desired, kpy, kiy, kdy, DIRECT);

float kpp2 = 0; //.45*kup;
float kip2 = 0; //.54*kup/tup;
float kdp2 = 0; //.54*kup*tup;
PID pitch2PID(&pitch, &pitch_angle2, &pitch_desired, kpp2, kip2, kdp2, DIRECT);
float kpr2 = 0.0;
float kir2 = 0.0;
float kdr2 = 0.0;
PID roll2PID(&roll, &roll_angle2, &roll_desired, kpr2, kir2, kdr2, DIRECT);
#endif
/*
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = true;

PID_ATune aTune(&pitch, &pitch1);
*/

// remote control vars
int input;
char c = ' ';

bool newData = false;

// servo objects
Servo hip1;
Servo shoulder1;
Servo knee1;
Servo hip2;
Servo shoulder2;
Servo knee2;
Servo hip3;
Servo shoulder3;
Servo knee3;
Servo hip4;
Servo shoulder4;
Servo knee4;

struct angles leg1;
struct angles leg2;
struct angles leg3;
struct angles leg4;

struct angles leg1_final;
struct angles leg2_final;
struct angles leg3_final;
struct angles leg4_final;

// IMU setup
MPU6050 mpu;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;

// setup and initialization code
void setup()
{
  // pin setup
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);

  // initialize servos
  hip1.attach(0);
  shoulder1.attach(1);
  knee1.attach(2);
  hip2.attach(3);
  shoulder2.attach(4);
  knee2.attach(5);
  hip3.attach(6);
  shoulder3.attach(7);
  knee3.attach(8);
  hip4.attach(9);
  shoulder4.attach(10);
  knee4.attach(11);

  digitalWrite(13, HIGH);
  // initialize serial bus
  Serial.begin(9600);
  delay(2000);
  Serial4.begin(9600); // 38400

  // set initial leg position

  x1.pos = 0;
  yy1.pos = 0;
  z1.pos = leg_height;
  x2.pos = x1.pos;
  yy2.pos = yy1.pos;
  z2.pos = z1.pos;
  x3.pos = x1.pos;
  yy3.pos = yy1.pos;
  z3.pos = z1.pos;
  x4.pos = x1.pos;
  yy4.pos = yy1.pos;
  z4.pos = z1.pos;
  offset();

  // velocities
  x1.vel = 0;
  yy1.vel = 0;
  z1.vel = 0;
  x2.vel = 0;
  yy2.vel = 0;
  z2.vel = 0;
  x3.vel = x1.vel_f;
  yy3.vel = -yy1.vel_f;
  z3.vel = z1.vel_f;
  x4.vel = x2.vel_f;
  yy4.vel = yy2.vel_f;
  z4.vel = z2.vel_f;

  // accelerations
  x1.accel = 0;
  yy1.accel = 0;
  z1.accel = 0;
  x2.accel = 0;
  yy2.accel = 0;
  z2.accel = 0;
  x3.accel = x1.accel;
  yy3.accel = -yy1.accel;
  z3.accel = z1.accel;
  x4.accel = x2.accel;
  yy4.accel = yy2.accel;
  z4.accel = z2.accel;

  x1.pos_f = 0;
  yy1.pos_f = 0;
  z1.pos_f = leg_height;
  x2.pos_f = x1.pos_f;
  yy2.pos_f = yy1.pos_f;
  z2.pos_f = z1.pos_f;
  x3.pos_f = x1.pos_f;
  yy3.pos_f = yy1.pos_f;
  z3.pos_f = z1.pos_f;
  x4.pos_f = x1.pos_f;
  yy4.pos_f = yy1.pos_f;
  z4.pos_f = z1.pos_f;
  offset();

  // velocities
  x1.vel_f = 0;
  yy1.vel_f = 0;
  z1.vel_f = 0;
  x2.vel_f = 0;
  yy2.vel_f = 0;
  z2.vel_f = 0;
  x3.vel_f = x1.vel_f;
  yy3.vel_f = -yy1.vel_f;
  z3.vel_f = z1.vel_f;
  x4.vel_f = x2.vel_f;
  yy4.vel_f = yy2.vel_f;
  z4.vel_f = z2.vel_f;

  // accelerations
  x1.accel_f = 0;
  yy1.accel_f = 0;
  z1.accel_f = 0;
  x2.accel_f = 0;
  yy2.accel_f = 0;
  z2.accel_f = 0;
  x3.accel_f = x1.accel_f;
  yy3.accel_f = -yy1.accel_f;
  z3.accel_f = z1.accel_f;
  x4.accel_f = x2.accel_f;
  yy4.accel_f = yy2.accel_f;
  z4.accel_f = z2.accel_f;

  // calculate inverse kinematics
  IK_interpolate2(t0, tf, t);

  leg1 = IK_final(x1.pos, yy1.pos, z1.pos);
  leg2 = IK_final(x2.pos, yy2.pos, z2.pos);
  leg3 = IK_final(x3.pos, yy3.pos, z3.pos);
  leg4 = IK_final(x4.pos, yy4.pos, z4.pos);

  // write angles to servos
  write_servos();

  // debug & calibrate servo angles
  /*
  leg1.hip = 0;
  leg1.shoulder = 45;
  leg1.knee = 90;
  leg2.hip = 0;
  leg2.shoulder = 45;
  leg2.knee = 90;
  leg3.hip = 0;
  leg3.shoulder = 45;
  leg3.knee = 90;
  leg4.hip = 0;
  leg4.shoulder = 45;
  leg4.knee = 90;
  */

  // IMU setup
  Serial.print("IMU setup start");
  delay(2500);
  // IMU_setup();
  Serial.print("IMU setup end");

  // sets # of points interpolated between servo positions: more = smoother
  interpolation_interval = state_time / n_interps;
  // PID start

#ifdef PID_BALANCE_ENABLE
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-20, 20);
  yawPID.SetSampleTime(10);

  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-20, 20);
  pitchPID.SetSampleTime(100);
  pitch2PID.SetMode(AUTOMATIC);
  pitch2PID.SetOutputLimits(-100, 100);
  pitch2PID.SetSampleTime(100);

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-20, 20);
  rollPID.SetSampleTime(100);
  roll2PID.SetMode(AUTOMATIC);
  roll2PID.SetOutputLimits(-30, 30);
  roll2PID.SetSampleTime(100);
#endif

  delay(5000);
}

void loop()
{
  t = millis();
  if (t - t_old_rc > 1)
  {
    t_old_rc = t;
    remoteIO();
  }
#ifdef DEBUG_TIMER
  // debug_timer(); // 1
#endif

// get IMU data (to add)
#ifdef IMU_ENABLE
  get_IMU_data();
  yaw = ypr[0] * 180 / M_PI;
  pitch = ypr[2] * 180 / M_PI;
  roll = ypr[1] * 180 / M_PI;
#endif

#ifdef DEBUG_TIMER
  // debug_timer(); // 2
#endif

#ifdef PID_BALANCE_ENABLE
  yawPID.Compute();
  yaw1 = constrain(yaw1, -10, 10);
  pitchPID.Compute();
  // pitch1 = constrain(pitch1,-10,10);
  rollPID.Compute();
#endif
#ifdef DEBUG_TIMER
  // debug_timer(); // 3
#endif
  // roll_angle1 = constrain(roll_angle1,-20,20);
  // roll2PID.Compute();
  roll_angle2 = constrain(roll_angle2, -40, 40);
  // pitch2PID.Compute();

  // PID_tune();

  // static test modes
  if ((mode == 0) || (mode == 2))
  {
    // timing control
    if (t - t_old_state >= state_time)
    {
      t_old_state = t;
      t0 = t;
      tf = t0 + state_time;

      // define leg positions
      x1.pos_f = x_dist;
      yy1.pos_f = steer2;
      z1.pos_f = leg_height;
      x2.pos_f = x1.pos_f;
      yy2.pos_f = yy1.pos_f;
      z2.pos_f = leg_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = steer2;
      z3.pos_f = leg_height;
      x4.pos_f = -x1.pos_f;
      yy4.pos_f = yy3.pos_f;
      z4.pos_f = leg_height;
      offset();

      // velocities
      x1.vel_f = 0;
      yy1.vel_f = 0;
      z1.vel_f = 0;
      x2.vel_f = 0;
      yy2.vel_f = 0;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = -yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = -yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    balance();
#ifdef DEBUG_TIMER
    debug_timer(); // 4
#endif
                   // interpolation timing
    if (t - t_old >= interpolation_interval)
    {
      t_old = t;

      // interpolate to final angles
      // IK_interpolate2(t - state_time/2, t + state_time/2, t);
      IK_interpolate2(t0, tf, t);

      leg1 = IK_final(x1.pos, yy1.pos, z1.pos);
      leg2 = IK_final(x2.pos, yy2.pos, z2.pos);
      leg3 = IK_final(x3.pos, yy3.pos, z3.pos);
      leg4 = IK_final(x4.pos, yy4.pos, z4.pos);
      // write angles to servos
      write_servos();

#ifdef PRINT_LEG_XYZ
      Serial.print(x_dist);
      Serial.print("\t");
      Serial.print(steer2);
      Serial.print("\t");
      Serial.print(x1.pos);
      Serial.print("\t");
      Serial.print(yy1.pos);
      Serial.print("\t");
      Serial.print(z1.pos);
      Serial.println("\t");
#endif

#ifdef DEBUG_TIMER
      debug_timer(); // 7
#endif
    }
  }
  // balance mode
  if (mode == 3)
  {
    // timing control
    if (t - t_old_state >= state_time)
    {
      t_old_state = t;
      t0 = t;
      tf = t0 + state_time;
      state = 5;
    }

    // define leg positions
    get_xyz();
#ifdef PID_BALANCE_ENABLE
    balance();
#endif

    // interpolation timing
    if (t - t_old >= interpolation_interval)
    {
      t_old = t;

      // interpolate to final angles
      IK_interpolate2(t0, tf, t);

      leg1 = IK_final(x1.pos, yy1.pos, z1.pos);
      leg2 = IK_final(x2.pos, yy2.pos, z2.pos);
      leg3 = IK_final(x3.pos, yy3.pos, z3.pos);
      leg4 = IK_final(x4.pos, yy4.pos, z4.pos);
      // write angles to servos
      write_servos();
    }
  }

  // walking mode
  if (mode == 1)
  {

    // states controller/timer
    if (t - t_old_state >= state_time)
    {
      t_old_state = t;
      t0 = t;
      tf = t0 + state_time;
      state++;

      if (state > N_STATES)
      {
        state = 1;
      }
      get_xyz();
    }

    // walking states - breaks step motion into 6 points (states)
    
#ifdef PID_BALANCE_ENABLE
    balance();
#endif

    // interpolation timing
    if (t - t_old >= interpolation_interval)
    {
      t_old = t;

      // debug

#ifdef PRINT_LEG_XYZ
      Serial.print(state);
      Serial.print("\t");
      Serial.print(x1.pos);
      Serial.print("\t");
      Serial.print(yy1.pos);
      Serial.print("\t");
      Serial.print(z1.pos);
      Serial.print("\t");
#endif

      // interpolate to final angles
      IK_interpolate2(t0, tf, t);
#ifdef DEBUG_TIMER
      debug_timer(); // 5
#endif

      leg1 = IK_final(x1.pos, yy1.pos, z1.pos);
      leg2 = IK_final(x2.pos, yy2.pos, z2.pos);
      leg3 = IK_final(x3.pos, yy3.pos, z3.pos);
      leg4 = IK_final(x4.pos, yy4.pos, z4.pos);
#ifdef DEBUG_TIMER
      debug_timer(); // 6
#endif

      // write angles to servos
      write_servos();

      // debug
#ifdef PRINT_LEG_THETA
      Serial.print(leg1.hip);
      Serial.print("\t");
      Serial.print(leg1.shoulder);
      Serial.print("\t");
      Serial.print(leg1.knee);
      Serial.print("\t");
#endif
#ifdef DEBUG_SERIAL_PRINT
      Serial.println("");
#endif
    }
  }
}