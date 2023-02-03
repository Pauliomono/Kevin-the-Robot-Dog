#ifndef CONFIGURATION_H
#define CONFIGURATION_H

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

//#define COMM_ROLE_VEHICLE
#define COMM_ROLE_CONTROLLER


//COMM mirror - enables mirroring serial communications to USB

/*
#define SERIAL_MIRROR
#define SERIAL_MIRROR_TIME
#define SERIAL_MIRROR_MNEMONIC
#define SERIAL_MIRROR_CHECKSUM
#define SERIAL_MIRROR_TYPE
#define SERIAL_MIRROR_DATA
*/

//#define SERIAL_MIRROR_VALID
#define COMM_SHOW_INTERPRETER

#define CHECKSUM_FAIL

#endif