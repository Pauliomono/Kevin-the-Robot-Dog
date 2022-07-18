#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PID_v1.h"


//code control vars
int state_time = 1000;
int interpolation_interval;
int n_interps = 20;
int state = 1;
int mode = 0;
int n_states = 6;

//timing vars
int t0;
int tf;
int t;
int t_old;
int t_old_state;

//foot position vars
float L = 124;
float l_pitch = 156;
float l_roll = 76;
float x1;
float yy1;
float z1;
float x2;
float yy2;
float z2;
float x3;
float yy3;
float z3;
float x4;
float yy4;
float z4;

//leg position control vars
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
float leg_height = 200;
int control_increment = 20;

double yaw;
double pitch;
double roll;

double yaw_desired = 0;
double pitch_desired = -2;//1
double roll_desired = 0;

//PID setup
float kup = 4.5;
float tup = 1*.01613;
float kpp = .05;//.45*kup;
float kip = 0.0;//.54*kup/tup;
float kdp = 0;//.54*kup*tup;
PID pitchPID(&pitch, &pitch_angle1, &pitch_desired, kpp, kip, kdp, DIRECT);
float kpr = .05;//.6;
float kir = 0.01;//.01;
float kdr = 0.01;//.04;
PID rollPID(&roll, &roll_angle1, &roll_desired, kpr, kir, kdr, DIRECT);
float kpy = 0.0;
float kiy = 0.0;
float kdy = 0.0;
PID yawPID(&yaw, &yaw1, &yaw_desired, kpy, kiy, kdy, DIRECT);

float kpp2 = 0;//.45*kup;
float kip2 = 0;//.54*kup/tup;
float kdp2 = 0;//.54*kup*tup;
PID pitch2PID(&pitch, &pitch_angle2, &pitch_desired, kpp2, kip2, kdp2, DIRECT);
float kpr2 = 0.0;
float kir2 = 0.0;
float kdr2 = 0.0;
PID roll2PID(&roll, &roll_angle2, &roll_desired, kpr2, kir2, kdr2, DIRECT);

/*
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = true;

PID_ATune aTune(&pitch, &pitch1);
*/

//remote control vars
int input;

bool newData = false;

//servo objects
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

//leg angle structure
struct angles{
  float hip;
  float shoulder;
  float knee;
};

struct angles leg1;
struct angles leg2;
struct angles leg3;
struct angles leg4;

struct angles leg1_final;
struct angles leg2_final;
struct angles leg3_final;
struct angles leg4_final;

//IMU setup
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

//setup and initialization code
void setup() {
  //pin setup 
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

  //initialize servos
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
  //initialize serial bus
  Serial.begin(9600);

  //set initial leg position
  x1 = 0;
  yy1 = 0;
  z1 = leg_height;
  x2 = x1;
  yy2 = yy1;
  z2 = z1;
  x3 = x1;
  yy3 = yy1;
  z3 = z1;
  x4 = x1;
  yy4 = yy1;
  z4 = z1;

  //calculate inverse kinematics
  leg1 = IK_final(x1, yy1, z1);
  leg2 = IK_final(x2, yy2, z2);
  leg3 = IK_final(x3, yy3, z3);
  leg4 = IK_final(x4, yy4, z4);

  //write angles to servos 
  write_servos();
  
  //debug & calibrate servo angles
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


  //IMU setup
  Serial.print("IMU setup start");
  delay(2500);
  IMU_setup();
  Serial.print("IMU setup end");

  //sets # of points interpolated between servo positions: more = smoother
  interpolation_interval = state_time/n_interps;
  //PID start
  
  
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
  
  delay(5000); 
}

void loop() {
  t = millis();

  //get serial input for remote control (currently over serial - to add: bluetooth)
  get_input();
  interpret_input();

  //get IMU data (to add)
  get_IMU_data();
  yaw = ypr[0] * 180/M_PI;
  pitch = ypr[2] * 180/M_PI;
  roll = ypr[1] * 180/M_PI;

  yawPID.Compute();
  yaw1 = constrain(yaw1,-10,10);
  pitchPID.Compute();
  //pitch1 = constrain(pitch1,-10,10);
  rollPID.Compute();
  //roll_angle1 = constrain(roll_angle1,-20,20);
  //roll2PID.Compute();
  roll_angle2 = constrain(roll_angle2,-40,40);
  //pitch2PID.Compute();

  //PID_tune();
              
  //static test modes
  if ((mode == 0)||(mode == 2)){ 
    
    //timing control 
    if (t - t_old_state >= state_time ){
      t_old_state = t;
      t0 = t;
      tf = t0 + state_time;
    }

    //define leg positions
      x1 = x_dist;
      yy1 = steer1;
      z1 = leg_height;
      x2 = x1;
      yy2 = yy1;
      z2 = leg_height;
      x3 = -x1;
      yy3 = steer2;
      z3 = leg_height;
      x4 = -x1;
      yy4 = yy3;
      z4 = leg_height;

      balance();
    //interpolation timing
    if(t - t_old >= interpolation_interval){
      t_old = t;

      //calculate inverse kinematics
      leg1_final = IK_final(x1, yy1, z1);
      leg2_final = IK_final(x2, yy2, z2);
      leg3_final = IK_final(x3, yy3, z3);
      leg4_final = IK_final(x4, yy4, z4);
      
      //interpolate to final angles
      leg1 = IK_interpolate(leg1_final, leg1, t0, tf, t);
      leg2 = IK_interpolate(leg2_final, leg2, t0, tf, t);
      leg3 = IK_interpolate(leg3_final, leg3, t0, tf, t);
      leg4 = IK_interpolate(leg4_final, leg4, t0, tf, t);

      //write angles to servos
      write_servos();
    }
  }
  //balance mode
  if (mode == 3){ 
    
    //timing control 
    if (t - t_old_state >= state_time ){
      t_old_state = t;
      t0 = t;
      tf = t0 + state_time;
      state == 5;
    }

    //define leg positions
      get_xyz();
      balance();
      
    //interpolation timing
    if(t - t_old >= interpolation_interval){
      t_old = t;

      //calculate inverse kinematics
      leg1_final = IK_final(x1, yy1, z1);
      leg2_final = IK_final(x2, yy2, z2);
      leg3_final = IK_final(x3, yy3, z3);
      leg4_final = IK_final(x4, yy4, z4);
      
      //interpolate to final angles
      leg1 = IK_interpolate(leg1_final, leg1, t0, tf, t);
      leg2 = IK_interpolate(leg2_final, leg2, t0, tf, t);
      leg3 = IK_interpolate(leg3_final, leg3, t0, tf, t);
      leg4 = IK_interpolate(leg4_final, leg4, t0, tf, t);

      //write angles to servos
      write_servos();
    }
  }

  //walking mode
  if (mode == 1){

  //states controller/timer
  if (t - t_old_state >= state_time ){
    t_old_state = t;
    t0 = t;
    tf = t0 + state_time;
    state++;

    if (state > n_states){
      state = 1;
    }
  }

  //walking states - breaks step motion into 6 points (states)
  get_xyz();

  balance();


  //interpolation timing
  if(t - t_old >= interpolation_interval){
  t_old = t;

  //debug
  /*
  Serial.print(state);
  Serial.print("  ");
  Serial.print(x1);
  Serial.print("  ");
  Serial.print(yy1);
  Serial.print("  ");
  Serial.print(z1);
  Serial.print("  ");
  Serial.println("  ");
  */

  //calculate inverse kinematics
  leg1_final = IK_final(x1, yy1, z1);
  leg2_final = IK_final(x2, yy2, z2);
  leg3_final = IK_final(x3, yy3, z3);
  leg4_final = IK_final(x4, yy4, z4);

  //interpolate to final angles
  leg1 = IK_interpolate(leg1_final, leg1, t0, tf, t);
  leg2 = IK_interpolate(leg2_final, leg2, t0, tf, t);
  leg3 = IK_interpolate(leg3_final, leg3, t0, tf, t);
  leg4 = IK_interpolate(leg4_final, leg4, t0, tf, t);

  //write angles to servos  
  write_servos();

//debug
  /*
  Serial.print(leg1_final.hip);
  Serial.print("  "); 
  Serial.print(leg1.hip);
  Serial.print("  ");  
  Serial.print(leg1_final.shoulder);
  Serial.print("  ");
  Serial.print(leg1.shoulder);
  Serial.print("  ");
  Serial.print(leg1_final.knee);
  Serial.print("  ");
  Serial.print(leg1.knee);
  Serial.print("  ");
  
  */
  
  }
  }
  
}

//inverse kinematics function - calculates leg angles from xyz foot coords
struct angles IK_final(float rx,float ry,float rz){
  struct angles theta;
  float r = sqrt(pow(rz,2) + pow(rx,2));
  
  theta.hip = 180/PI*atan2(ry,rz); 
  theta.knee = 180/PI*(acos((2*pow(L,2) - pow(r,2))/(2*L*L)));
  theta.shoulder = 180/PI*(acos((pow(r,2))/(2*r*L)) - atan2(rx,rz));

  return(theta);
}

//interpolation function - gets angles between current and final angle
struct angles IK_interpolate(struct angles leg_final, struct angles leg, int t0, int tf,int t){
  struct angles theta;
  
  //interpolate angles based on time left for state 
  theta.hip = interpolate2(leg_final.hip, leg.hip, t0, tf, t); 
  theta.shoulder = interpolate2(leg_final.shoulder, leg.shoulder, t0, tf, t); 
  theta.knee = interpolate2(leg_final.knee, leg.knee, t0, tf, t);
  
  return(theta);
}

//simple interpolation function - constant velocity
float interpolate2(float thetaf, float theta, int t0, int tf, int t){
  tf = tf-t0; 
  t = t-t0;
    
  int n_intervals = (tf - t)/interpolation_interval;
  if(n_intervals == 0){
    n_intervals = 1;
  }
  float dtheta = (float)(thetaf - theta)/n_intervals;
  theta = dtheta + theta;
  return(theta);
}

//cubic spline interpolation function - not currently working
/*
float interpolate1(float thetaf, float theta0, int t0, int tf, int t){
  tf = tf-t0; 
  t = t-t0;
    
  //compute a0-a3
  float a0 = theta0;
  float a1 = 0;
  float a2 = (float)3*(thetaf - theta0)/pow(tf,2);
  float a3 = (float)-2/pow(tf,3)*(thetaf - theta0);
    
  float theta = (float)a0 + a1*t + a2*pow(t,2) + a3*pow(t,3);
  return(theta);
}
*/

//grab input function (for remote control)
void get_input() {
        input = Serial.read();
}

//interpreter function - takes inputs and parses out robot control
void interpret_input() {
  if((mode == 0)||(mode == 1)){
      //forward/backward
        if(input == 'w'){
          x_dist = x_dist - control_increment;
        }
        if(input == 's'){
          x_dist = x_dist + control_increment;
        }
      //strafe left/right
        if(input == 'a'){
          steer1 = steer1 - control_increment;
          steer2 = steer2 + control_increment;
        }
        if(input == 'd'){
          steer1 = steer1 + control_increment;
          steer2 = steer2 - control_increment;
        }
  }
  if((mode == 1)||(mode == 2)){
      //yaw left/right
        if(input == 'q'){
          steer1 = steer1 - control_increment;
          steer2 = steer2 - control_increment;
        }
        if(input == 'e'){
          steer1 = steer1 + control_increment;
          steer2 = steer2 + control_increment;
        }
  }
  if(mode == 0){
        if(input == 'q'){
          leg_height = leg_height + control_increment;

        }
        if(input == 'e'){
          leg_height = leg_height - control_increment;

        }
  }
  if(mode == 2){
         //pitch up
        if(input == 'w'){
          pitch1 = pitch1 - control_increment;
          pitch2 = pitch2 + control_increment;
        }
        //pitch down
        if(input == 's'){
          pitch1 = pitch1 + control_increment;
          pitch2 = pitch2 - control_increment;
        }
        //roll left
        if(input == 'a'){
          roll1 = roll1 - control_increment;
          roll2 = roll2 + control_increment;
        }
        //roll right
        if(input == 'd'){
          roll1 = roll1 + control_increment;
          roll2 = roll2 - control_increment;
        }
        
  }
        //select mode
        if(input == '0'){
          //static translation
          mode = 0;
          state_time = 1000;
          interpolation_interval = state_time/n_interps;
        }
        if(input == '1'){
          //walking
          mode = 1;
          state_time = 75;
          step_height = 20;
          leg_height = 200;
          interpolation_interval = state_time/n_interps;
        }
        if(input == '2'){
          //static rpy
          mode = 2;
          state_time = 1000;
          interpolation_interval = state_time/n_interps;
        }

        //adjust control sensitivity
        //trimming mode
        if(input == 't'){
          control_increment = 1;
        }
        //movement mode
        if(input == 'y'){
          control_increment = 20;
        }
        if(input == '3'){
          //walking
          mode = 3;
          state_time = 250;
          step_height = 80;
          leg_height = 200;
          interpolation_interval = state_time/n_interps;
        }
}

void IMU_setup(){
      // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void get_IMU_data() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            //Serial.print(euler[0] * 180/M_PI);
            //Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
           
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);

            Serial.print("\t");
            Serial.println(pitch_angle2);

            
            
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
    }
}
/*
void PID_tune(){
  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      pitchPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
}
*/

void balance(){
  //add rpy offsets - will be implemented into balance control
  /*
  pitch1 = l_pitch*sin(pitch_angle1/180*PI);
  roll1 = l_roll*sin(roll_angle1/180*PI);
  yaw2 = -l_pitch*sin(yaw1/180*PI);
  */
  pitch1 = leg_height*sin(pitch_angle1/180*PI);
  roll1 = leg_height*sin(roll_angle1/180*PI);

  x1 = x1 - pitch1;
  x2 = x2 - pitch1;
  x3 = x3 + pitch1;
  x4 = x4 + pitch1;

  yy1 = yy1 + roll1;
  yy2 = yy2 - roll1;
  yy3 = yy3 + roll1;
  yy4 = yy4 + roll1;
  /*
  z1 = z1 - pitch1 - roll1;
  z2 = z2 - pitch1 + roll1;
  z3 = z3 + pitch1 + roll1;
  z4 = z4 + pitch1 - roll1;
  */
  
  pitch2 = sin(pitch_angle2/180*PI);

  /*
  if((state == 1)||(state == 2)||(state == 3)){
  
  //yy1 = yy1 - z1*roll2;
  yy2 = yy2 + z2*roll_angle2;
  //yy3 = yy3 + z3*roll2;
  yy4 = yy4 - z4*roll_angle2;

  yy2 = constrain(yy2, -100, 10);
  yy4 = constrain(yy4, -100, 10);

  }

  if((state == 4)||(state == 5)||(state == 6)){
  
  yy1 = yy1 - z1*roll_angle2;
  //yy2 = yy2 + z2*roll_angle2;
  yy3 = yy3 - z3*roll_angle2;
  //yy4 = yy4 - z4*roll_angle2;

  yy1 = constrain(yy1, -100, 10);
  yy3 = constrain(yy3, -10, 100);

  }
  */
}

void write_servos(){
  hip1.write(85 + leg1.hip);
  shoulder1.write(96 + leg1.shoulder);
  knee1.write(180 + 2 - leg1.knee);
  hip2.write(95 + leg2.hip);
  shoulder2.write(90 - leg2.shoulder);
  knee2.write(3 + leg2.knee);
  hip3.write(93 + leg3.hip);
  shoulder3.write(96 + leg3.shoulder);
  knee3.write(180 + 5 - leg3.knee);
  hip4.write(91 + leg4.hip);
  shoulder4.write(95 - leg4.shoulder);
  knee4.write(10 + leg4.knee);
}

void get_xyz(){
  //walking states - breaks step motion into 6 points (states)
  if(state == 1){
    x1 = x_dist;
    yy1 = steer2;
    z1 = leg_height;
    x2 = (float).5*-x_dist;
    yy2 = (float).5*steer2;
    z2 = leg_height - .5*step_height;
    x3 = -x1;
    yy3 = -steer2;
    z3 = z1;
    x4 = -x2;
    yy4 = (float).5*steer2;
    z4 = z2;
  }
  if(state == 2){
    x1 = 0;
    yy1 = 0;
    z1 = leg_height;
    x2 = 0;
    yy2 = 0;
    z2 = leg_height - step_height;;
    x3 = -x1;
    yy3 = 0;
    z3 = z1;
    x4 = -x2;
    yy4 = 0;
    z4 = z2;
  }
  if(state == 3){
    x1 = -x_dist;
    yy1 = -steer2;
    z1 = leg_height;
    x2 = (float).5*x_dist;
    yy2 = (float).5*-steer2;
    z2 = leg_height - .5*step_height;
    x3 = -x1;
    yy3 = steer2;
    z3 = 200;
    x4 = -x2;
    yy4 = (float).5*-steer2;
    z4 = z2;
  }
  if(state == 4){
    x1 = (float).5*-x_dist;
    yy1 = (float).5*-steer2;
    z1 = leg_height - .5*step_height;
    x2 = x_dist;
    yy2 = -steer2;
    z2 = leg_height;
    x3 = -x1;
    yy3 = (float).5*steer2;
    z3 = z1;
    x4 = -x2;
    yy4 = -steer2;
    z4 = z2;
  }
  if(state == 5){
    x1 = 0;
    yy1 = 0;
    z1 = leg_height - step_height;;
    x2 = 0;
    yy2 = 0;
    z2 = leg_height;
    x3 = -x1;
    yy3 = 0;
    z3 = z1;
    x4 = -x2;
    yy4 = 0;
    z4 = z2;
  }
  if(state == 6){
    x1 = (float).5*x_dist;
    yy1 = (float).5*steer2;
    z1 = leg_height - .5*step_height;
    x2 = -x_dist;
    yy2 = steer2;
    z2 = leg_height;
    x3 = -x1;
    yy3 = (float).5*-steer2;
    z3 = z1;
    x4 = -x2;
    yy4 = steer2;
    z4 = z2;
  }
}
