#include <Servo.h>

float L = 124;
float theta_knee = 0;
float theta_shoulder1 = 0;
float theta_shoulder2 = 0;
int state_time = 125;
int interpolation_interval;
int n_interps = 20;
int state = 1;
int n_states = 8;
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
float x_dist = 80;


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
void setup() {
  

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
  pinMode(14, OUTPUT);

  hip1.attach(0);
  shoulder1.attach(1);
  knee1.attach(2);
  hip2.attach(3);
  shoulder2.attach(4);
  knee2.attach(14);
  hip3.attach(6);
  shoulder3.attach(7);
  knee3.attach(8);
  hip4.attach(9);
  shoulder4.attach(10);
  knee4.attach(11);

  digitalWrite(13, HIGH);
  Serial.begin(9600);

  interpolation_interval = state_time/n_interps;  
  
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
  shoulder2.write(85 - leg2.shoulder);
  knee2.write(3 + leg2.knee);
  hip3.write(93 + leg3.hip);
  shoulder3.write(96 + leg3.shoulder);
  knee3.write(180 + 5 - leg3.knee);
  hip4.write(91 + leg4.hip);
  shoulder4.write(95 - leg4.shoulder);
  knee4.write(5 + leg4.knee);
  
  delay(5000);
  

  
}

void loop() {
  t = millis();
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
    x1 = 0;
    yy1 = 0;
    z1 = 100;
  }
  if(state == 2){
    x1 = 0;
    yy1 = 0;
    z1 = 125;
  }
  if(state == 3){
    x1 = 0;
    yy1 = 0;
    z1 = 150;
  }
  if(state == 4){
    x1 = 0;
    yy1 = 0;
    z1 = 175;
  }
  if(state == 5){
    x1 = 0;
    yy1 = 0;
    z1 = 200;
  }
  if(state == 6){
    x1 = 0;
    yy1 = 0;
    z1 = 175;
  }
  if(state == 7){
    x1 = 0;
    yy1 = 0;
    z1 = 150;
  }
  if(state == 8){
    x1 = 0;
    yy1 = 0;
    z1 = 125;
  }
  
// not much here yet
  if(t - t_old >= interpolation_interval){
  t_old = t;
  
  leg1 = IK(x1, yy1, z1, t0, tf, leg1.hip, leg1.shoulder, leg1.knee);
  leg2 = IK(x1, yy1, z1, t0, tf, leg2.hip, leg2.shoulder, leg2.knee);
  leg3 = IK(x1, yy1, z1, t0, tf, leg3.hip, leg3.shoulder, leg3.knee);
  leg4 = IK(x1, yy1, z1, t0, tf, leg4.hip, leg4.shoulder, leg4.knee);
  Serial.print(leg1.hip);
  Serial.print("  ");
  
  
  hip1.write(90 + leg1.hip);
  shoulder1.write(94 + leg1.shoulder);
  knee1.write(180 + 2 - leg1.knee);
  hip2.write(95 + leg2.hip);
  shoulder2.write(85 - leg2.shoulder);
  knee2.write(3 + leg2.knee);
  hip3.write(93 + leg3.hip);
  shoulder3.write(96 + leg3.shoulder);
  knee3.write(180 + 5 - leg3.knee);
  hip4.write(91 + leg4.hip);
  shoulder4.write(95 - leg4.shoulder);
  knee4.write(5 + leg4.knee);

  /*
  knee1.writeMicroseconds(deg2ms(180 - leg1.knee));
  Serial.print(leg1.knee);
  Serial.print("  ");
  shoulder11.writeMicroseconds(deg2ms(leg1.shoulder1));
  Serial.print(leg1.shoulder1);
  Serial.print("  ");
  shoulder12.writeMicroseconds(deg2ms(45 + leg1.shoulder2));
  Serial.println(leg1.shoulder2);
  */
  Serial.println("  ");
  }
}

struct angles IK(float rx,float ry,float rz, int t0, int tf, float theta_hip, float theta_shoulder, float theta_knee){
  struct angles theta;
  float r = sqrt(pow(rz,2) + pow(rx,2));

  float theta_hip_f = 180/PI*atan2(ry,rz); 
  float theta_knee_f = 180/PI*(acos((2*pow(L,2) - pow(r,2))/(2*L*L)));
  float theta_shoulder_f = 180/PI*(acos((pow(r,2))/(2*r*L)) - atan2(rx,rz));

  /*
  Serial.print(theta_hip_f);
  Serial.print("  ");
  Serial.print(theta_shoulder_f);
  Serial.print("  ");
  Serial.print(theta_knee_f);
  Serial.print("  ");
  */
  
  int t = millis();
  
  theta.hip = interpolate2(theta_hip_f, theta_hip, t0, tf, t); 
  theta.shoulder = interpolate2(theta_shoulder_f, theta_shoulder, t0, tf, t); 
  theta.knee = interpolate2(theta_knee_f, theta_knee, t0, tf, t);
  /* 
  Serial.print(theta.hip);
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
  int n_intervals = (tf - t)/interpolation_interval + 1;
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

float deg2ms(float degrees)
{
  return 1000.0 + degrees * 50.0/9.0;
}
