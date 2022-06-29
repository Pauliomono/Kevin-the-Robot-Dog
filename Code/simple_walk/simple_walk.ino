#include <Servo.h>

float L = 124;
float theta_knee = 0;
float theta_shoulder1 = 0;
float theta_shoulder2 = 0;
int state_time = 50;
int interpolation_interval;
int n_interps = 20;
int state = 1;
int mode = 0;
int n_states = 6;
int t0;
int tf;
int t;
int t_old;
int t_old_state;
float theta_hip_0 = 0;
float theta_shoulder_0 = 0;
float theta_knee_0 = 0;

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

float x_dist = 0;
float steer1 = 0;
float steer2 = 0;
float step_height = 15;
float leg_height = 200;

int input;
int control_increment = 20;
bool newData = false;

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
// twelve servo objects can be created on most boards

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

  //servo initialize
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
  Serial.begin(9600);

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

  leg1 = IK_final(x1, yy1, z1);
  leg2 = IK_final(x2, yy2, z2);
  leg3 = IK_final(x3, yy3, z3);
  leg4 = IK_final(x4, yy4, z4);

  Serial.print(leg1.hip);
  Serial.print("  ");
  Serial.print(leg1.shoulder);
  Serial.print("  ");
  Serial.print(leg1.knee);
  Serial.println("  ");
  
  hip1.write(90 + leg1.hip);
  shoulder1.write(94 + leg1.shoulder);
  knee1.write(180 + 2 - leg1.knee);
  hip2.write(95 + leg2.hip);
  shoulder2.write(93 - leg2.shoulder);
  knee2.write(3 + leg2.knee);
  hip3.write(93 + leg3.hip);
  shoulder3.write(96 + leg3.shoulder);
  knee3.write(180 + 5 - leg3.knee);
  hip4.write(91 + leg4.hip);
  shoulder4.write(95 - leg4.shoulder);
  knee4.write(5 + leg4.knee);

 

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

  hip1.write(90 + leg1.hip);
  shoulder1.write(94 + leg1.shoulder);
  knee1.write(180 + 2 - leg1.knee);
  hip2.write(95 + leg2.hip);
  shoulder2.write(93 - leg2.shoulder);
  knee2.write(3 + leg2.knee);
  hip3.write(93 + leg3.hip);
  shoulder3.write(96 + leg3.shoulder);
  knee3.write(180 + 5 - leg3.knee);
  hip4.write(91 + leg4.hip);
  shoulder4.write(95 - leg4.shoulder);
  knee4.write(5 + leg4.knee);
  */
  
  
  interpolation_interval = state_time/n_interps;
  delay(5000); 
}

void loop() {
  t = millis();
  get_input();
  interpret_input();

  
  if (t - t_old_state >= state_time ){
    t_old_state = t;
    t0 = t;
    tf = t0 + state_time;
    state++;
    theta_hip_0 = hip1.read();
    theta_shoulder_0 = shoulder1.read();
    theta_knee_0 = knee1.read();
    if (state > n_states){
      state = 1;
    }
  }
  if(state == 1){
    x1 = x_dist;
    yy1 = steer1;
    z1 = leg_height;
    x2 = (float).5*-x_dist;
    yy2 = (float).5*-steer1;
    z2 = leg_height - .5*step_height;
    x3 = -x1;
    yy3 = steer2;
    z3 = z1;
    x4 = -x2;
    yy4 = (float).5*-steer2;
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
    yy1 = -steer1;
    z1 = leg_height;
    x2 = (float).5*x_dist;
    yy2 = (float).5*steer1;
    z2 = leg_height - .5*step_height;;
    x3 = -x1;
    yy3 = -steer2;
    z3 = 200;
    x4 = -x2;
    yy4 = (float).5*steer2;
    z4 = z2;
  }
  if(state == 4){
    x1 = (float).5*-x_dist;
    yy1 = (float).5*-steer1;
    z1 = leg_height - .5*step_height;
    x2 = x_dist;
    yy2 = steer1;
    z2 = leg_height;
    x3 = -x1;
    yy3 = (float).5*-steer2;
    z3 = z1;
    x4 = -x2;
    yy4 = steer2;
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
    yy1 = (float).5*steer1;
    z1 = leg_height - .5*step_height;;
    x2 = -x_dist;
    yy2 = -steer1;
    z2 = leg_height;
    x3 = -x1;
    yy3 = (float).5*steer2;
    z3 = z1;
    x4 = -x2;
    yy4 = -steer2;
    z4 = z2;
  }
  
// not much here yet
  if(t - t_old >= interpolation_interval){
  t_old = t;
  /*
  Serial.print(state);
  Serial.print("  ");
  Serial.print(x1);
  Serial.print("  ");
  Serial.print(yy1);
  Serial.print("  ");
  Serial.print(z1);
  Serial.print("  ");
  */
  leg1_final = IK_final(x1, yy1, z1);
  leg2_final = IK_final(x2, yy2, z2);
  leg3_final = IK_final(x3, yy3, z3);
  leg4_final = IK_final(x4, yy4, z4);

  leg1 = IK_interpolate(leg1_final, leg1, t0, tf, t);
  leg2 = IK_interpolate(leg2_final, leg2, t0, tf, t);
  leg3 = IK_interpolate(leg3_final, leg3, t0, tf, t);
  leg4 = IK_interpolate(leg4_final, leg4, t0, tf, t);
  //Serial.print(leg2.shoulder);
  //Serial.print("  ");

  
  
  hip1.write(90 + leg1.hip);
  shoulder1.write(94 + leg1.shoulder);
  knee1.write(180 + 2 - leg1.knee);
  hip2.write(95 + leg2.hip);
  shoulder2.write(93 - leg2.shoulder);
  knee2.write(3 + leg2.knee);
  hip3.write(93 + leg3.hip);
  shoulder3.write(96 + leg3.shoulder);
  knee3.write(180 + 5 - leg3.knee);
  hip4.write(91 + leg4.hip);
  shoulder4.write(95 - leg4.shoulder);
  knee4.write(5 + leg4.knee);

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
  
  Serial.println("  ");
  }
}

struct angles IK_final(float rx,float ry,float rz){
  struct angles theta;
  float r = sqrt(pow(rz,2) + pow(rx,2));
  
  theta.hip = 180/PI*atan2(ry,rz); 
  theta.knee = 180/PI*(acos((2*pow(L,2) - pow(r,2))/(2*L*L)));
  theta.shoulder = 180/PI*(acos((pow(r,2))/(2*r*L)) - atan2(rx,rz));

  
  /*Serial.print(theta_hip_f);
  Serial.print("  ");
  Serial.print(theta_shoulder_f);
  Serial.print("  ");
  Serial.print(theta_knee_f);
  Serial.print("  ");
  */
  
  
  /*Serial.print(theta.hip);
  Serial.print("  ");  
  Serial.print(theta.shoulder);
  Serial.print("  "); 
  Serial.print(theta.knee);
  Serial.print("  "); 
  /*
  theta.knee = theta_knee_f; 
  theta.shoulder1 = theta_shoulder1_f; 
  theta.shoulder2 = 180 - theta_shoulder2_f;
  */

  return(theta);
}

struct angles IK_interpolate(struct angles leg_final, struct angles leg, int t0, int tf,int t){
  struct angles theta;
  
  /*Serial.print(theta_hip_f);
  Serial.print("  ");
  Serial.print(theta_shoulder_f);
  Serial.print("  ");
  Serial.print(theta_knee_f);
  Serial.print("  ");
  */
  
  theta.hip = interpolate2(leg_final.hip, leg.hip, t0, tf, t); 
  theta.shoulder = interpolate2(leg_final.shoulder, leg.shoulder, t0, tf, t); 
  theta.knee = interpolate2(leg_final.knee, leg.knee, t0, tf, t);
  
  /*Serial.print(theta.hip);
  Serial.print("  ");  
  Serial.print(theta.shoulder);
  Serial.print("  "); 
  Serial.print(theta.knee);
  Serial.print("  "); 
  /*
  theta.knee = theta_knee_f; 
  theta.shoulder1 = theta_shoulder1_f; 
  theta.shoulder2 = 180 - theta_shoulder2_f;
  */

  return(theta);
}

float interpolate2(float thetaf, float theta, int t0, int tf, int t){
  tf = tf-t0; 
  t = t-t0;
    
  //compute a0-a3
  int n_intervals = (tf - t)/interpolation_interval;
  if(n_intervals == 0){
    n_intervals = 1;
  }
  float dtheta = (float)(thetaf - theta)/n_intervals;
  theta = dtheta + theta;
  return(theta);
}

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

void get_input() {
        input = Serial.read();
}

void interpret_input() {
      //forward/backward
        if(input == 'w'){
          x_dist = x_dist - control_increment;
        }
        if(input == 's'){
          x_dist = x_dist + control_increment;
        }
      //strafe left/right
        if(input == 'a'){
          steer1 = steer1 + control_increment;
          steer2 = steer2 - control_increment;
        }
        if(input == 'd'){
          steer1 = steer1 - control_increment;
          steer2 = steer2 + control_increment;
        }
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
