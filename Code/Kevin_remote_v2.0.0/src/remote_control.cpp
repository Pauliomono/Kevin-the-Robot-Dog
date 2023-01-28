#include <remote_control.h>

/*
Packet list: all telemetry items sent out + their mnemonics
Each includes a time stamp
-time (for sync purposes?) 1  KEVTIME
-motor angles (target + actual) 12 (24) S1A_TAR S1A_ACT
-leg coords (target + actual) (24) L1X_POS L1Y_POS L1Z_POS L1XFPOS L1YFPOS L1ZFPOS
-rpy (3) ROLLVAL PIT_VAL YAW_VAL
-xyz accel (3) ACCEL_X ACCEL_Y ACCEL_Z
-mode (1) MODEVAL
-balance mode (1) BALMODE
-PID values (3) PIDVALP PIDVALI PIDVALD
-command positions (translate + yaw) (3) FWDMOVE SIDMOVE YAWMOVE

Mnemonic list
MH1A_TAR - leg 1 hip target angle
MH1A_ACT - leg 1 hip actual angle
MS1A_TAR - leg 1 shoulder target angle
MS1A_ACT - leg 1 shoulder actual angle
MK1A_TAR - leg 1 knee target angle
MK1A_ACT - leg 1 knee actual angle
MH2A_TAR - leg 2 hip target angle
MH2A_ACT - leg 2 hip actual angle
MS2A_TAR - leg 2 shoulder target angle
MS2A_ACT - leg 2 shoulder actual angle
MK2A_TAR - leg 2 knee target angle
MK2A_ACT - leg 2 knee actual angle
MH3A_TAR - leg 3 hip target angle
MH3A_ACT - leg 3 hip actual angle
MS3A_TAR - leg 3 shoulder target angle
MS3A_ACT - leg 3 shoulder actual angle
MK3A_TAR - leg 3 knee target angle
MK3A_ACT - leg 3 knee actual angle
MH4A_TAR - leg 4 hip target angle
MH4A_ACT - leg 4 hip actual angle
MS4A_TAR - leg 4 shoulder target angle
MS4A_ACT - leg 4 shoulder actual angle
MK4A_TAR - leg 4 knee target angle
MK4A_ACT - leg 4 knee actual angle
ML1X_POS - leg 1 x position
ML1Y_POS - leg 1 Y position
ML1Z_POS - leg 1 Z position
ML1XFPOS - leg 1 x final position
ML1YFPOS - leg 1 Y final position
ML1ZFPOS - leg 1 Z final position
ML2X_POS - leg 2 x position
ML2Y_POS - leg 2 Y position
ML2Z_POS - leg 2 Z position
ML2XFPOS - leg 2 x final position
ML2YFPOS - leg 2 Y final position
ML2ZFPOS - leg 2 Z final position
ML3X_POS - leg 3 x position
ML3Y_POS - leg 3 Y position
ML3Z_POS - leg 3 Z position
ML3XFPOS - leg 3 x final position
ML3YFPOS - leg 3 Y final position
ML3ZFPOS - leg 3 Z final position
ML4X_POS - leg 4 x position
ML4Y_POS - leg 4 Y position
ML4Z_POS - leg 4 Z position
ML4XFPOS - leg 4 x final position
ML4YFPOS - leg 4 Y final position
ML4ZFPOS - leg 4 Z final position
MROLLVAL - roll value
MPIT_VAL - pitch value
MYAW_VAL - yaw value
MACCEL_X - x acceleration
MACCEL_Y - y acceleration
MACCEL_Z - z acceleration
MMODEVAL - mode that Kevin is in
MBALMODE - type of balance being used
MPIDVLRP - PID roll proportional value
MPIDVLRI - PID roll integral value
MPIDVLRD - PID roll derivative value
MPIDVLPP - PID pitch proportional value
MPIDVLPI - PID pitch integral value
MPIDVLPD - PID pitch derivative value
MFWDMOVE - move forward value
MSIDMOVE - move sideways value
MYAWMOVE - rotate in place value
MSTEPSTE - state in walking motion

Mnemonics to add:
BATTLVL - Battery charge percentage
PCKT_TX - # of packets sent (may help if there's data loss)
PCKT_RX - # of packets recieved (may help if there's data loss)
*/

/*
63 mnemonics

115 bits per millisecond
each character is 8 bits
14 bytes per millisecond

T0000000MXXXXXXXD0000000E (2 milliseconds per packet)

1Hz data (1000ms)
PIDVLRP - PID roll proportional value
PIDVLRI - PID roll integral value
PIDVLRD - PID roll derivative value
PIDVLPP - PID pitch proportional value
PIDVLPI - PID pitch integral value
PIDVLPD - PID pitch derivative value
MODEVAL - mode that Kevin is in
BALMODE - type of balance being used
KEVTIME - time onboard Kevin (milliseconds)

4Hz data (250ms)
FWDMOVE - move forward value
SIDMOVE - move sideways value
YAWMOVE - rotate in place value
ACCEL_X - x acceleration
ACCEL_Y - y acceleration
ACCEL_Z - z acceleration
YAW_VAL - yaw value

40Hz data (25ms)
ROLLVAL - roll value
PIT_VAL - pitch value
STEPSTE - state in walking motion
H1A_TAR - leg 1 hip target angle
S1A_TAR - leg 1 shoulder target angle
K1A_TAR - leg 1 knee target angle
L1X_POS - leg 1 x position
L1Y_POS - leg 1 Y position
L1Z_POS - leg 1 Z position
*/

#ifdef COMM_MODE_DESCRIPTIVE

void comms_send()
{
  int comm_time = millis();

  if (remainder(comm_time, 25) == 0)
  {
    // ROLLVAL - IMU roll value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("ROLLVAL");
    Serial4.print("D");
    Serial4.print(roll);
    Serial4.print("E");
    // PIT_VAL - IMU pitch value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("PIT_VAL");
    Serial4.print("D");
    Serial4.print(pitch);
    Serial4.print("E");
    // STEPSTE - state in walking motion
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("STEPSTE");
    Serial4.print("I");
    Serial4.print(state);
    Serial4.print("E");
    // H1A_TAR - leg 1 hip target angle
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("H1A_TAR");
    Serial4.print("F");
    Serial4.print(leg1.hip);
    Serial4.print("E");
    // S1A_TAR - leg 1 shoulder target angle
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("S1A_TAR");
    Serial4.print(leg1.shoulder);
    Serial4.print("F");
    Serial4.print("E");
    // K1A_TAR - leg 1 knee target angle
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("K1A_TAR");
    Serial4.print(leg1.knee);
    Serial4.print("F");
    Serial4.print("E");
    // L1X_POS - leg 1 x position
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("L1X_POS");
    Serial4.print("F");
    Serial4.print(x1.pos);
    Serial4.print("E");
    // L1Y_POS - leg 1 Y position
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("L1Y_POS");
    Serial4.print("F");
    Serial4.print(yy1.pos);
    Serial4.print("E");
    // L1Z_POS - leg 1 Z position
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("L1Z_POS");
    Serial4.print("F");
    Serial4.print(z1.pos);
    Serial4.print("E");
  }

  // 4Hz data (250ms)
  if (remainder(comm_time, 250) == 18)
  {
    // FWDMOVE - move forward value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("FWDMOVE");
    Serial4.print("F");
    Serial4.print(x_dist);
    Serial4.print("E");
    // SIDMOVE - move sideways value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("SIDMOVE");
    Serial4.print("F");
    Serial4.print(steer2); // placeholder
    Serial4.print("E");
  }

  if (remainder(comm_time, 250) == 43)
  {
    // YAWMOVE - rotate in place value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("YAWMOVE");
    Serial4.print("F");
    Serial4.print(steer2);
    Serial4.print("E");
    // YAW_VAL - IMU yaw value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("YAW_VAL");
    Serial4.print("D");
    Serial4.print(yaw);
    Serial4.print("E");
  }

  if (remainder(comm_time, 250) == 68)
  {
    // ACCEL_X - x acceleration
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("ACCEL_X");
    Serial4.print("D");
    Serial4.print(x_accel);
    Serial4.print("E");
    // ACCEL_Y - y acceleration
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("ACCEL_Y");
    Serial4.print("D");
    Serial4.print(y_accel);
    Serial4.print("E");
    // ACCEL_Z - z acceleration
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("ACCEL_Z");
    Serial4.print("D");
    Serial4.print(z_accel);
    Serial4.print("E");
  }

  // 1Hz data (1000ms)
  if (remainder(comm_time, 1000) == 343)
  {
    // PIDVLRP - PID roll proportional value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("PIDVLRP");
    Serial4.print("F");
    Serial4.print(kpr);
    Serial4.print("E");
    // PIDVLRI - PID roll integral value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("PIDVLRI");
    Serial4.print("F");
    Serial4.print(kir);
    Serial4.print("E");
    // PIDVLRD - PID roll derivative value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("PIDVLRD");
    Serial4.print("F");
    Serial4.print(kdr);
    Serial4.print("E");
  }

  if (remainder(comm_time, 1000) == 368)
  {
    // PIDVLPP - PID pitch proportional value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("PIDVLPP");
    Serial4.print("F");
    Serial4.print(kpp);
    Serial4.print("E");
    // PIDVLPI - PID pitch integral value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("PIDVLPI");
    Serial4.print("F");
    Serial4.print(kip);
    Serial4.print("E");
    // PIDVLPD - PID pitch derivative value
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("PIDVLPD");
    Serial4.print("F");
    Serial4.print(kdp);
    Serial4.print("E");
  }

  if (remainder(comm_time, 1000) == 393)
  {
    // MODEVAL - mode that Kevin is in
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("MODEVAL");
    Serial4.print("I");
    Serial4.print(mode);
    Serial4.print("E");
    // BALMODE - type of balance being used
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("BALMODE");
    Serial4.print("I");
    Serial4.print(mode); // placeholder
    Serial4.print("E");
    // KEVTIME - time onboard Kevin (milliseconds)
    Serial4.print("T");
    Serial4.print(millis());
    Serial4.print("KEVTIME");
    Serial4.print("I");
    Serial4.print(millis());
    Serial4.print("E");
  }
}

void comms_receive()
{

  /*
  1. if 1 byte available, parse it
    if character parses as "T", start recording, go to 2
  2. capture time - record time (int)
    if 4 bytes in buffer, read/store as int
    save in working var for time stamp, go to 3
  3. capture mnemonic - capture 7 more characters
    if 7 bytes in buffer, read/store as 7 char string
    save in working var for mnemonic
  4. read data type: bool, int, float, double
    determine how to read the data
  5-8. capture value - record characters based on data type
    read # or bytes based on specified datatype
  9. return result
    save working vars to complete packet var
    set new data flag to true



  */

  // 1. if 1 byte available, parse it
  if (comm_receive_step == 1)
  {
    if (Serial.available() > 0)
    {
      char start_byte;
      start_byte = Serial4.read();
      if (start_byte == 'T')
      {
        comm_receive_step++;
      }
    }
  }
  // 2. capture time - record time (int)
  if (comm_receive_step == 2)
  {
    if (Serial.available() >= 4)
    {
      time_stamp = Serial.parseInt();
      comm_receive_step++;
    }
  }
  // 3. capture mnemonic - capture 7 more characters
  if (comm_receive_step == 3)
  {
    if (Serial.available() >= 7)
    {
      for (int i = 0; i < 7; i++)
      {
        mnemonic += Serial4.read();
      }
      comm_receive_step++;
    }
  }

  // 4. capture datatype - bool, int, float, double

  if (comm_receive_step == 4)
  {
    if (Serial.available() >= 1)
    {
      char data_byte;
      data_byte = Serial4.read();
      if (data_byte == 'B')
      {
        comm_receive_step = 5;
      }
      if (data_byte == 'I')
      {
        comm_receive_step = 6;
      }
      if (data_byte == 'F')
      {
        comm_receive_step = 7;
      }
      if (data_byte == 'D')
      {
        comm_receive_step = 8;
      }
    }
  }

  // 5-8. capture value - record characters until character is "E"
  // bool
  if (comm_receive_step == 5)
  {
    if (Serial.available() >= 2)
    {
      comm_data.b = (Serial4.read() << 8) + Serial4.read();
      comm_results.time = time_stamp;
      comm_results.mnemonic = mnemonic;
      comm_results.data.b = comm_data.b;
      comm_receive_step = 9;
    }
  }
  // int
  if (comm_receive_step == 6)
  {
    if (Serial.available() >= 4)
    {
      comm_data.i = Serial.parseInt();
      comm_results.time = time_stamp;
      comm_results.mnemonic = mnemonic;
      comm_results.data.i = comm_data.i;
      comm_receive_step = 9;
    }
  }
  // float
  if (comm_receive_step == 7)
  {
    if (Serial.available() >= 4)
    {
      comm_data.f = Serial.parseFloat();
      comm_results.time = time_stamp;
      comm_results.mnemonic = mnemonic;
      comm_results.data.f = comm_data.f;
      comm_receive_step = 9;
    }
  }
  // double
  if (comm_receive_step == 7)
  {
    if (Serial.available() >= 8)
    {
      Serial4.readBytes(double_buffer, 8);
      memcpy(&comm_data.d, double_buffer, sizeof(comm_data.d));

      comm_results.time = time_stamp;
      comm_results.mnemonic = mnemonic;
      comm_results.data.d = comm_data.d;
      comm_receive_step = 9;
    }
  }
  // 9. return results
  if (comm_receive_step == 9)
  {
    if (Serial.available() >= 1)
    {
      char end_byte = Serial4.read(); // check if E?
      comm_receive_step = 1;
      new_data = 1;
    }
  }
}

/*
comms interpreter
- updates variables based on mnemonics received
- compare received mnemonic against list
- update var for match
- clear new_data flag
*/

void comms_interpreter()
{
  if (new_data > 0)
  {
#ifdef COMM_ROLE_VEHICLE
    if (comm_results.mnemonic == 'FWDMOVE')
    {
      x_dist = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'SIDMOVE')
    {
      steer2 = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'YAWMOVE')
    {
      steer2 = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'MODEVAL')
    {
      mode = comm_results.data.i;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLRP')
    {
      kpr = comm_results.data.f;
      rollPID.SetTunings(kpr, kir, kdr);
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLRI')
    {
      kir = comm_results.data.f;
      rollPID.SetTunings(kpr, kir, kdr);
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLRD')
    {
      kdr = comm_results.data.f;
      rollPID.SetTunings(kpr, kir, kdr);
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLPP')
    {
      kpp = comm_results.data.f;
      pitchPID.SetTunings(kpp, kip, kdp);
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLPI')
    {
      kip = comm_results.data.f;
      pitchPID.SetTunings(kpp, kip, kdp);
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLPD')
    {
      kdp = comm_results.data.f;
      pitchPID.SetTunings(kpp, kip, kdp);
      new_data = 0;
    }

#endif

#ifdef COMM_ROLE_CONTROLLER
    if (comm_results.mnemonic == "ROLLVAL")
    {
      roll = comm_results.data.d;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIT_VAL')
    {
      pitch = comm_results.data.d;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'YAW_VAL')
    {
      yaw = comm_results.data.d;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'ACCEL_X')
    {
      x_accel = comm_results.data.d;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'ACCEL_Y')
    {
      y_accel = comm_results.data.d;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'ACCEL_Z')
    {
      z_accel = comm_results.data.d;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'STEPSTE')
    {
      state = comm_results.data.i;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'H1A_TAR')
    {
      leg1.hip = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'S1A_TAR')
    {
      leg1.shoulder = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'K1A_TAR')
    {
      leg1.knee = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'L1X_POS')
    {
      x1.pos = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'L1Y_POS')
    {
      yy1.pos = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'L1Z_POS')
    {
      z1.pos = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLRP')
    {
      kpr = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLRI')
    {
      kir = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLRD')
    {
      kdr = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLPP')
    {
      kpp = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLPI')
    {
      kip = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'PIDVLPD')
    {
      kdp = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'BALMODE')
    {
      // bal_mode = comm_results.data.i;
      new_data = 0;
    }

    if (comm_results.mnemonic == 'KEVTIME')
    {
      // time = comm_results.data.i;
      new_data = 0;
    }
#endif
  }
}
#endif