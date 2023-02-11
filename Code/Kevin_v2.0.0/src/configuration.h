#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define SERIAL_BUFFER_SIZE 2048
extern uint8_t serial_buffer[SERIAL_BUFFER_SIZE];

/*--------------------------------------------------------------------------
Enable/disable any features/modes for Kevin
--------------------------------------------------------------------------*/

#define PID_BALANCE_ENABLE
//#define INPUT_SHAPING
#define IMU_ENABLE



#define NMODES 3

/*--------------------------------------------------------------------------
Remote Control Settings
--------------------------------------------------------------------------*/

/*comms send/recieve method: 
(INT and STRING are really unsophisticated - probably gonna remove once descriptive COMMs works)
INT - telemetry is coded into a single integer - fastest but error prone, limited telemetry
STRING - telemetry is coded into a string - no errors, limited telemetry
DESCRIPTIVE - telemetry includes time, Mnemonic, data format & data - most structured and easier to send LOTS of data types
*/

//#define COMM_MODE_INT
//#define COMM_MODE_STRING
#define COMM_MODE_DESCRIPTIVE

//COMMs role - tells comm functions if this is the controller or the vehicle

#define COMM_ROLE_VEHICLE
//#define COMM_ROLE_CONTROLLER

//COMM mirror - enables mirroring serial communications to USB, primarily a debug tool

/*
#define SERIAL_MIRROR
#define SERIAL_MIRROR_TIME
#define SERIAL_MIRROR_MNEMONIC
#define SERIAL_MIRROR_CHECKSUM
#define SERIAL_MIRROR_TYPE
#define SERIAL_MIRROR_DATA
*/

//#define SERIAL_MIRROR_VALID

//COMM interpreter results
#define COMM_SHOW_INTERPRETER
//#define CHECKSUM_FAIL


//determine high rate telemetry types - only 1 20hz data type atm

//#define SEND_ANGLES
//#define SEND_COORDS
#define SEND_RPY

/*--------------------------------------------------------------------------
Select walking gait types: 6 state, 8 state
--------------------------------------------------------------------------*/

//don't pick 6 state, super jank
#define N_STATES 8

/*--------------------------------------------------------------------------
Select PWM type: internal comes off Teensy, external off breakout board
--------------------------------------------------------------------------*/

//#define SERVO_PWM_INTERNAL
#define SERVO_PWM_EXTERNAL

/*--------------------------------------------------------------------------
Enable any debug stuff to print data to serial terminal
--------------------------------------------------------------------------*/
#define DEBUG_SERIAL_PRINT

#ifdef DEBUG_SERIAL_PRINT

//#define PRINT_TIME
//#define DEBUG_TIMER
//#define PRINT_LEG_XYZ
//#define PRINT_LEG_THETA
//#define PRINT_INTERPOLATION
//#define REMOTE_DATA_PRINT

#endif

#ifdef IMU_ENABLE

//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL


#ifdef DEBUG_SERIAL_PRINT
//#define IMU_PRINT
#endif

#endif

#endif