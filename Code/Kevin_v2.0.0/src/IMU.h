#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <configuration.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

void dmpDataReady();
void IMU_setup();
void get_IMU_data();

#endif