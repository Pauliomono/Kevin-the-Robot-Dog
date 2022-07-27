#ifndef ASSEMBLE_INPUT_H
#define ASSEMBLE_INPUT_H

#include <Arduino.h>
#include <interpreter.h>
#include <configuration.h>

extern int input;
extern bool data_recieved;
extern String datapacket;
extern int packet;
extern int packet2;

void remoteIO();
void get_input();
void send_telemetry();
void assemble_input();

#endif