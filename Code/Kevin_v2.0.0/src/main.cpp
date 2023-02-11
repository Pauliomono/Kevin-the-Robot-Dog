#include <Arduino.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

#include <Adafruit_PWMServoDriver.h>

#include "interpolation.h"
#include <remote_control.h>
#include "IMU.h"
#include <servos.h>
#include <kinematics.h>
#include <balance_controller.h>
#include <debugging.h>
#include <configuration.h>
#include <Smoothed.h>

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
int state_time;
int interpolation_interval = 20; // keep this a constant - servos won't update quicker than 20ms
int n_interps;
int state = 1;
int mode = 0;
int n_states = 6;

// timing vars
float t0 = 0;
float tf = 1000;
int t;
int t_old;
int t_old_state;
int comm_timer;
int imu_timer;
int mode_timer;

// SoftwareSerial Remote (21, 20);
String datapacket;
bool data_recieved = 0;
int xxx_old = 0;
int yyy_old = 0;
int tune_precision = 0;
float x_tune = 0;
float y_tune = 0;
int tune_mode = 0;

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
struct r shoulders;
// these all need to be measured
float L_body = 100;
float L_shoulder = 100;
float L_COM = 100;
float roll_input = 0;
float pitch_input = 0;
float yaw_input = 0;

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
double x_accel;
double y_accel;
double z_accel;

double yaw_target = 0;
double pitch_target = 0;
double roll_target = 0;
double yaw_commanded = 0;
double pitch_commanded = 0;
double roll_commanded = 0;

#ifdef PID_BALANCE_ENABLE
// PID setup
float kpp = 0.0; //.45*kup;
float kip = 0.0; //.54*kup/tup;
float kdp = 0.0; //.54*kup*tup;
PID pitchPID(&pitch, &pitch_commanded, &pitch_target, kpp, kip, kdp, DIRECT);

float kpr = 0.0; //.05;
float kir = 0.0; //.01;
float kdr = 0.0; //.01;
PID rollPID(&roll, &roll_commanded, &roll_target, kpr, kir, kdr, DIRECT);

float kpy = 0.0;
float kiy = 0.0;
float kdy = 0.0;
PID yawPID(&yaw, &yaw_commanded, &yaw_target, kpy, kiy, kdy, DIRECT);

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
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Smoothed<int> hip_feedback_angle;

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

// PWM board setup
const int OscillatorFrequency = 25725000;
const int PWMFreq = 50;
const int startup_delay = 5000;

// input shaper globals
double frequency;
double amplitude;
int root_time;
int root_time_counter; // global?
double low_trigger;
double high_trigger;
double peak;
double trough;
int peak_time;
int trough_time;
int root_time_1; // global?
int root_time_2;
bool trigger_state; // global?
int step;           // global
int roots_delta;
double mpeak;
double mtrough;
int t_sin_init;
int phase_time;

// comms globals
int comm_receive_step = 1;
int time_stamp;
uint16_t checksum;
char mnemonic[7];
data_type comm_data;
char double_buffer[8];
comm comm_results;
bool new_data;
char data_byte;
char telem_packet[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t serial_buffer[SERIAL_BUFFER_SIZE];

void setup() // setup and initialization code
{
      // initialize serial bus
      Serial.begin(115200);
      delay(2000);
      Serial4.begin(115200);
      Serial4.addMemoryForWrite(serial_buffer, SERIAL_BUFFER_SIZE);
      Serial.println("Kevin v2.0.0");
      Serial.println("-----------------------------------------------------------------");
      Serial.println("Begin Kevin startup:");
      Serial.println("-----------------------------------------------------------------");

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
      digitalWrite(13, HIGH);
      pinMode(A6, INPUT);
      hip_feedback_angle.begin(SMOOTHED_AVERAGE, 8);
      hip_feedback_angle.add(analogRead(A6));

      Serial.println("Pin Modes Set");

#ifdef SERVO_PWM_INTERNAL
      // initialize servos
      Serial.println("PWM signaling set to internal");
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
      Serial.println("Servo motors attached");
#endif

#ifdef SERVO_PWM_EXTERNAL
      // initialize pwm board
      Serial.println("PWM signaling set to external");
      pwm.begin();
      pwm.setOscillatorFrequency(OscillatorFrequency); // 25725000
      pwm.setPWMFreq(PWMFreq);                         // 50
      Serial.print("PWM oscillator frequency set to ");
      Serial.print(OscillatorFrequency);
      Serial.println(" Hz");
      Serial.print("PWM frequency set to ");
      Serial.print(PWMFreq);
      Serial.println(" Hz");
      Serial.println("PWM setup complete");
#endif

      state = 0;
      get_xyz();
      // set initial leg position
      Serial.println("Initializing leg kinematics:");
      shoulders = attitude_model(roll_commanded, pitch_commanded, yaw_commanded);
      x1.pos = x1.pos_f;
      yy1.pos = yy1.pos_f;
      z1.pos = z1.pos_f;
      x2.pos = x2.pos_f;
      yy2.pos = yy2.pos_f;
      z2.pos = z2.pos_f;
      x3.pos = x3.pos_f;
      yy3.pos = yy3.pos_f;
      z3.pos = z3.pos_f;
      x4.pos = x4.pos_f;
      yy4.pos = yy4.pos_f;
      z4.pos = z4.pos_f;
      Serial.println("leg 0 position set");

      // velocities
      x1.vel = x1.vel_f;
      yy1.vel = yy1.vel_f;
      z1.vel = z1.vel_f;
      x2.vel = x2.vel_f;
      yy2.vel = yy2.vel_f;
      z2.vel = z2.vel_f;
      x3.vel = x1.vel;
      yy3.vel = -yy1.vel;
      z3.vel = z1.vel;
      x4.vel = x2.vel;
      yy4.vel = yy2.vel;
      z4.vel = z2.vel;
      Serial.println("leg 0 velocity set");

      // accelerations
      x1.accel = x1.accel_f;
      yy1.accel = yy1.accel_f;
      z1.accel = z1.accel_f;
      x2.accel = x2.accel_f;
      yy2.accel = yy2.accel_f;
      z2.accel = z2.accel_f;
      x3.accel = x1.accel;
      yy3.accel = -yy1.accel;
      z3.accel = z1.accel;
      x4.accel = x2.accel;
      yy4.accel = yy2.accel;
      z4.accel = z2.accel;
      Serial.println("leg 0 acceleration set");

      Serial.println("leg final acceleration set");
      Serial.println("leg kinematics initialization complete.");

      // calculate inverse kinematics
      IK_interpolate2(t0, tf, t);
      Serial.println("Initial kinematics calculated");

      leg1 = IK_final(x1.pos, yy1.pos, z1.pos);
      leg2 = IK_final(x2.pos, yy2.pos, z2.pos);
      leg3 = IK_final(x3.pos, yy3.pos, z3.pos);
      leg4 = IK_final(x4.pos, yy4.pos, z4.pos);

      // write angles to servos
      write_servos();
      Serial.println("Legs enable");

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
      Serial.println("IMU setup start");
      delay(2500);
      IMU_setup();
      Serial.println("IMU setup complete");

      // sets # of points interpolated between servo positions: more = smoother
      // edit - currently locked servo updates to 50hz due to servo limitations
      n_interps = state_time / interpolation_interval;
      // PID start

#ifdef PID_BALANCE_ENABLE
      // yawPID.SetMode(AUTOMATIC);
      yawPID.SetOutputLimits(-20, 20);
      // yawPID.SetSampleTime(10);

      // pitchPID.SetMode(AUTOMATIC);
      pitchPID.SetOutputLimits(-20, 20);
      // pitchPID.SetSampleTime(10);

      rollPID.SetMode(AUTOMATIC);
      rollPID.SetOutputLimits(-20, 20);
      rollPID.SetSampleTime(10);
#endif

#ifdef INPUT_SHAPING
      Serial.println("Starting input shaper");
      step = 0;
      int n_sample_count = 0;
      int n_sample_size = 60; // must be greater than 1 and even
      while (n_sample_count < n_sample_size)
      {
            hip_feedback_angle.add(analogRead(A6));
            Serial.print((float)-15 / 44 * (hip_feedback_angle.get() - 491) + 90);
            Serial.print("\t");
            Serial.print(90 + leg3.hip);
            Serial.print("\t");
            Serial.print(millis());
            Serial.print("\t");
            Serial.print(roots_delta);
            Serial.print("\t");
            Serial.print(frequency);
            Serial.print("\t");
            Serial.print(amplitude);
            Serial.print("\t");
            Serial.println(n_sample_count);

            n_sample_count = input_shape_setup(90 + leg3.hip, (float)-15 / 44 * (hip_feedback_angle.get() - 491) + 90, n_sample_count);
            delay(2);
      }
      Serial.print("Determined frequency: ");
      Serial.print(frequency);
      Serial.println(" Hz");
      Serial.print("Determined amplitude: +/-");
      Serial.print(amplitude);
      Serial.println(" deg");

      // frequency = 6;
      // amplitude = 4;
#endif

      Serial.println("-----------------------------------------------------------------");
      Serial.print("Kevin startup complete. Pausing for ");
      Serial.print(startup_delay);
      Serial.println(" milliseconds.");
      Serial.println("-----------------------------------------------------------------");
      delay(startup_delay);
}

void loop()
{
      t = millis();
#ifdef COMM_MODE_STRING
      remoteIO();
#endif

#ifdef COMM_MODE_INT
      remoteIO();
#endif

#ifdef COMM_MODE_DESCRIPTIVE

      if (comm_timer != t)
      {
            comm_timer = t;
            comms_send();
      }

      comms_receive();
      comms_interpreter();
#endif

#ifdef DEBUG_TIMER
      // debug_timer(); // 1
#endif

// get IMU data (to add)
#ifdef IMU_ENABLE

      if (imu_timer != t)
      {
            get_IMU_data();
            yaw = ypr[0] * 180 / M_PI;
            pitch = ypr[2] * 180 / M_PI;
            roll = (double)ypr[1] * 180 / M_PI;
            x_accel = aaWorld.x;
            y_accel = aaWorld.y;
            z_accel = aaWorld.z;
            imu_timer = t;
      }

#endif

#ifdef DEBUG_TIMER
      // debug_timer(); // 2
#endif

#ifdef DEBUG_TIMER
      // debug_timer(); // 3
#endif
      // roll_angle1 = constrain(roll_angle1,-20,20);
      // roll2PID.Compute();
      roll_angle2 = constrain(roll_angle2, -40, 40);
      // pitch2PID.Compute();

      // PID_tune();

      // mode state timing control
      if ((t % 250 == 0) && (mode_timer != t))
      {
            if (mode == 0)
            {
                  state_time = 1000;
                  n_interps = state_time / interpolation_interval;
            }
            if (mode == 1)
            {
                  state_time = 100;
                  n_interps = state_time / interpolation_interval;
            }
            if (mode == 2)
            {
                  state_time = 100;
                  n_interps = state_time / interpolation_interval;
            }
            if (mode == 3)
            {
                  state_time = 1000;
                  n_interps = state_time / interpolation_interval;
            }
            if (mode == 4)
            {
                  state_time = 1000;
                  n_interps = state_time / interpolation_interval;
            }
            mode_timer = t;
      }

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
                  state = 0;
                  get_xyz();
            }

#ifdef DEBUG_TIMER
            debug_timer(); // 4
#endif
                           // interpolation timing
            if (t - t_old >= interpolation_interval)
            {
                  t_old = t;
#ifdef PID_BALANCE_ENABLE
                  get_xyz();
#endif

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
                  state = 3;
                  // define leg positions
                  get_xyz();
            }

            // interpolation timing
            if (t - t_old >= interpolation_interval)
            {
                  t_old = t;
#ifdef PID_BALANCE_ENABLE
                  get_xyz();
#endif

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

            // interpolation timing
            if (t - t_old >= interpolation_interval)
            {
                  t_old = t;
#ifdef PID_BALANCE_ENABLE
                  get_xyz();
#endif
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

      if (mode == 4)
      {
            // timing control
            if (t - t_old_state >= state_time)
            {
                  t_old_state = t;
                  t0 = t;
                  tf = t0 + state_time;
                  state = 0;
                  // define leg positions
                  get_xyz();
            }

            // interpolation timing
            if (t - t_old >= interpolation_interval)
            {
                  t_old = t;
#ifdef PID_BALANCE_ENABLE
                  get_xyz();
#endif

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
}