#include <Servo.h>

float L = 124;
float theta_knee = 0;
float theta_shoulder1 = 0;
float theta_shoulder2 = 0;
int state_time = 1000;
int interpolation_interval;
int n_interps = 10;
int state = 1;
int n_states = 8;
int t0;
int tf;
int t;
int t_old;
int t_old_state;
float theta_knee_0 = 0;
float theta_shoulder1_0 = 0;
float theta_shoulder2_0 = 0;
float x1;
float yy1;
float z1;
float x_dist = 80;


Servo shoulder11;
Servo shoulder12;
Servo knee1;  // create servo object to control a servo
// twelve servo objects can be created on most boards

struct angles{
  float knee;
  float shoulder1;
  float shoulder2;
};

struct angles leg1;

void setup() {
  knee1.attach(9);
  shoulder11.attach(10);
  shoulder12.attach(11);
  Serial.begin(9600);
  knee1.write(90);
  shoulder11.write(180);

  interpolation_interval = state_time/n_interps;
  
  leg1.knee = 90;
  leg1.shoulder1 = 90;
  leg1.shoulder2 = 90;
  knee1.writeMicroseconds(deg2ms(leg1.knee));
  shoulder11.writeMicroseconds(deg2ms(leg1.shoulder1));
  shoulder12.writeMicroseconds(deg2ms(leg1.shoulder2));
  
  delay(50000);
  

  
}

void loop() {
  t = millis();
  if (t - t_old_state >= state_time ){
    t_old_state = t;
    t0 = t;
    tf = t0 + state_time;
    state++;
    theta_knee_0 = knee1.read();
    theta_shoulder1_0 = shoulder11.read();
    theta_shoulder2_0 = shoulder12.read();
    if (state > n_states){
      state = 1;
    }
  }
  if(state == 1){
    x1 = -x_dist;
    yy1 = 0;
    z1 = 180;
  }
  if(state == 2){
    x1 = 0;
    yy1 = 0;
    z1 = 100;
  }
  if(state == 3){
    x1 = x_dist;
    yy1 = 0;
    z1 = 180;
  }
  if(state == 4){
    x1 = (float)2/3*x_dist;
    yy1 = 0;
    z1 = 180;
  }
  if(state == 5){
    x1 = (float)1/3*x_dist;
    yy1 = 0;
    z1 = 180;
  }
  if(state == 6){
    x1 = 0;
    yy1 = 0;
    z1 = 180;
  }
  if(state == 7){
    x1 = (float)-1/3*x_dist;
    yy1 = 0;
    z1 = 180;
  }
  if(state == 8){
    x1 = (float)-2/3*x_dist;
    yy1 = 0;
    z1 = 180;
  }
  
// not much here yet
  if(t - t_old >= interpolation_interval){
  t_old = t;
  Serial.print(x1);
  Serial.print("  ");  
  leg1 = IK(x1, yy1, z1, t0, tf, leg1.knee, leg1.shoulder1, leg1.shoulder2);

  knee1.writeMicroseconds(deg2ms(180 - leg1.knee));
  Serial.print(leg1.knee);
  Serial.print("  ");
  shoulder11.writeMicroseconds(deg2ms(leg1.shoulder1));
  Serial.print(leg1.shoulder1);
  Serial.print("  ");
  shoulder12.writeMicroseconds(deg2ms(45 + leg1.shoulder2));
  Serial.println(leg1.shoulder2);
  }
}

struct angles IK(float rx,float ry,float rz, int t0, int tf, float theta_knee, float theta_shoulder1, float theta_shoulder2){
  struct angles theta;
  float r = sqrt(pow(rz,2) + pow(rx,2));

  float theta_shoulder1_f = 180/PI*atan2(ry,rz); 
  float theta_knee_f = 180/PI*(acos((2*pow(L,2) - pow(r,2))/(2*L*L)));
  float theta_shoulder2_f = 180/PI*(acos((pow(r,2))/(2*r*L)) - atan2(rx,rz));
  Serial.print(theta_knee_f);
  Serial.print("  ");
  Serial.print(theta_shoulder1_f);
  Serial.print("  ");
  Serial.print(theta_shoulder2_f);
  Serial.print("  ");
  
  int t = millis();
  
  theta.knee = interpolate2(theta_knee_f, theta_knee, t0, tf, t); 
  theta.shoulder1 = interpolate2(theta_shoulder1_f, theta_shoulder1, t0, tf, t); 
  theta.shoulder2 = interpolate2(theta_shoulder2_f, theta_shoulder2, t0, tf, t); 
  
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
