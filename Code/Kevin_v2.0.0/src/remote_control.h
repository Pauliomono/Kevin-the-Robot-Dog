#ifndef ASSEMBLE_INPUT_H
#define ASSEMBLE_INPUT_H

#include <Arduino.h>
#include <interpreter.h>
#include <configuration.h>
#include <servos.h>
#include <kinematics.h>

extern int t;
extern int input;
extern bool data_recieved;
extern String datapacket;
extern int packet;
extern int packet2;

// comms vars
extern double yaw;
extern double pitch;
extern double roll;
extern double yaw_target;
extern double pitch_target;
extern double roll_target;
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
    uint16_t checksum1;
    uint16_t checksum2;
    int time;
    String mnemonic;
    char type;
    union data_type data;
  };

extern int comm_receive_step;
extern int time_stamp;
extern char mnemonic[7];
extern union data_type comm_data;
extern struct comm comm_results;
extern bool new_data;
extern char data_byte;
extern char telem_packet[24];

template <class TYPE>
void packet_builder(char mnemon[7], char type, TYPE data_value);
void comms_send();
void comms_receive();
void comms_interpreter();

#endif