#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/*------------------------------------------------------------
Enable/disable any features/modes for Kevin
------------------------------------------------------------*/

//#define PID_BALANCE_ENABLE
#define IMU_ENABLE

//remote control send/recieve method: INT is faster but error prone
//#define RC_MODE_INT
#define RC_MODE_STRING

#define NMODES 2

/*------------------------------------------------------------
Select walking gait types: 6 state, 8 state
------------------------------------------------------------*/

#define N_STATES 8

/*------------------------------------------------------------
Enable any debug stuff to print data to serial terminal
------------------------------------------------------------*/
#define DEBUG_SERIAL_PRINT

#ifdef DEBUG_SERIAL_PRINT

//#define PRINT_TIME
//#define DEBUG_TIMER
#define PRINT_LEG_XYZ
//#define PRINT_LEG_THETA
//#define PRINT_INTERPOLATION
//#define REMOTE_DATA_PRINT_RAW
//#define REMOTE_DATA_PRINT

#endif

#ifdef IMU_ENABLE

//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL


#ifdef DEBUG_SERIAL_PRINT
#define IMU_PRINT
#endif

#endif

#endif