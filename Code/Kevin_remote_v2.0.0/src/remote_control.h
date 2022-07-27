#ifndef NUM2STR_H
#define NUM2STR_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#define NMODES 4

extern int mode;
extern SoftwareSerial SerialBT;

void remoteIO(int X, int Y, bool b1, bool b2, bool b3, bool b4);
void send_inputs_int(int X, int Y, bool b1, bool b2, bool b3, bool b4);
void send_inputs_string(int X, int Y, bool b1, bool b2, bool b3, bool b4);
String bool2str(bool X);
String num2str(int X);
void get_telemetry();


#endif