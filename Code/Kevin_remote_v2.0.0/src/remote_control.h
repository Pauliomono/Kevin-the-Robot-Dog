#ifndef ASSEMBLE_INPUT_H
#define ASSEMBLE_INPUT_H

#include <Arduino.h>
#include <interpreter.h>
#include <configuration.h>

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

//remote only header items
//leg angle structure
struct angles{
  float hip;
  float shoulder;
  float knee;
};

struct points{
  float pos;
  float vel;
  float accel;
  float pos_f;
  float vel_f;
  float accel_f;
};

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
    uint16_t checksum1;
    uint16_t checksum2;
    int time;
    String mnemonic;
    union data_type data;
  };

extern int comm_receive_step;
extern int time_stamp;
//extern uint16_t checksum1;
//extern uint16_t checksum2;
extern char mnemonic[7];
extern union data_type comm_data;
//extern byte double_buffer[8];//8?
extern struct comm comm_results;
extern bool new_data;
extern char data_byte;

void comms_send();
void comms_receive();
void comms_interpreter();

#endif