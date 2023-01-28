#ifndef ASSEMBLE_INPUT_H
#define ASSEMBLE_INPUT_H

#include <Arduino.h>
#include <interpreter.h>
#include <configuration.h>
#include <servos.h>
#include <kinematics.h>

extern float t;
extern int input;
extern bool data_recieved;
extern String datapacket;
extern int packet;
extern int packet2;

// comms vars
extern double yaw;
extern double pitch;
extern double roll;
extern int state;
extern struct angles leg1;
extern struct points x1;
extern struct points yy1;
extern struct points z1;
extern float x_dist;
extern float steer2;
extern double x_accel;
extern double y_accel;
extern double z_accel;
extern float kpp;
extern float kip;
extern float kdp;
extern float kpr;
extern float kir;
extern float kdr;



// comms receive globals
union data_type
  {
    bool b;
    int i;
    float f;
    double d;
  };
struct comm
  {
    int time;
    String mnemonic;
    union data_type data;
  };

extern int comm_receive_step;
extern int time_stamp;
extern String mnemonic;
extern union data_type comm_data;
extern char double_buffer[8];
extern struct comm comm_results;
extern bool new_data;

void remoteIO();
void get_input();
void send_telemetry(float X, float Y);
void assemble_input();
String dec2str(float X);
String num2str(int X);
String bool2str(bool X);
void comms_send();
void comms_receive();
void comms_interpreter();

#endif