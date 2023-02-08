#include <remote_control.h>

template <class TYPE>
void packet_builder(char mnemon[7], char type, TYPE data_value)
{
  char time_bytes[4];
  byte mnemonic_bytes[7];
  byte type_byte[1];
  byte data_bytes[8];
  u_int16_t checksum = 0; // 2 bytes

  int comm_time = millis();

  memcpy(time_bytes, &comm_time, sizeof comm_time);
  memcpy(mnemonic_bytes, &mnemon, sizeof mnemonic_bytes);
  memcpy(type_byte, &type, sizeof type);
  memcpy(data_bytes, &data_value, sizeof data_value);

  for (int i = 0; i < 4; i++)
  {
    checksum = checksum + time_bytes[i];
  }
  for (int i = 0; i < 7; i++)
  {
    checksum = checksum + mnemon[i];
  }

  checksum = checksum + type;
  for (int i = 0; i < 8; i++)
  {
    checksum = checksum + data_bytes[i];
  }

  byte checksum_bytes[2];
  memcpy(checksum_bytes, &checksum, sizeof checksum);

  // extern char telem_packet[24];

  telem_packet[0] = '<';
  telem_packet[1] = checksum_bytes[0];
  telem_packet[2] = checksum_bytes[1];
  telem_packet[3] = time_bytes[0];
  telem_packet[4] = time_bytes[1];
  telem_packet[5] = time_bytes[2];
  telem_packet[6] = time_bytes[3];
  telem_packet[7] = mnemon[0];
  telem_packet[8] = mnemon[1];
  telem_packet[9] = mnemon[2];
  telem_packet[10] = mnemon[3];
  telem_packet[11] = mnemon[4];
  telem_packet[12] = mnemon[5];
  telem_packet[13] = mnemon[6];
  telem_packet[14] = type;
  telem_packet[15] = data_bytes[0];
  telem_packet[16] = data_bytes[1];
  telem_packet[17] = data_bytes[2];
  telem_packet[18] = data_bytes[3];
  telem_packet[19] = data_bytes[4];
  telem_packet[20] = data_bytes[5];
  telem_packet[21] = data_bytes[6];
  telem_packet[22] = data_bytes[7];
  telem_packet[23] = '>';
}

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

40Hz data (25ms) (350 bytes)
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

#ifdef SEND_RPY
  if ((comm_time % 50) == 0)
  {
    // Serial.println(Serial4.availableForWrite());
    // ROLLVAL - IMU roll value
    packet_builder("ROLLVAL", 'd', roll);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 50) == 8)
  {
    // PIT_VAL - IMU pitch value
    packet_builder("PIT_VAL", 'd', pitch);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 50) == 16)
  {
    // YAW_VAL - IMU yaw value
    packet_builder("YAW_VAL", 'd', yaw);
    Serial4.write(telem_packet, 24);
  }
#endif

  if ((comm_time % 50) == 24)
  {
    // STEPSTE - state in walking motion
    packet_builder("STEPSTE", 'i', state);
    Serial4.write(telem_packet, 24);
  }

#ifdef SEND_ANGLES
  if ((comm_time % 50) == 0)
  {
    // H1A_TAR - leg 1 hip target angle
    packet_builder("H1A_TAR", 'f', leg1.hip);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 50) == 8)
  {
    // S1A_TAR - leg 1 shoulder target angle
    packet_builder("S1A_TAR", 'f', leg1.shoulder);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 50) == 16)
  {
    // K1A_TAR - leg 1 knee target angle
    packet_builder("K1A_TAR", 'f', leg1.knee);
    Serial4.write(telem_packet, 24);
  }
#endif

#ifdef SEND_COORDS
  if ((comm_time % 50) == 0)
  {
    // H1A_TAR - leg 1 hip target angle
    packet_builder("L1X_POS", 'f', x1.pos);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 50) == 8)
  {
    // S1A_TAR - leg 1 shoulder target angle
    packet_builder("L1Y_POS", 'f', yy1.pos);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 50) == 16)
  {
    // K1A_TAR - leg 1 knee target angle
    packet_builder("L1Z_POS", 'f', z1.pos);
    Serial4.write(telem_packet, 24);
  }
#endif

  // 4Hz data (250ms)

  if ((comm_time % 250) == 24)
  {
    // FWDMOVE - move forward value
    packet_builder("FWDMOVE", 'f', x_dist_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 250) == 32)
  {
    // SIDMOVE - move sideways value
    packet_builder("SIDMOVE", 'f', steer2_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 250) == 74)
  {
    // YAWMOVE - rotate in place value
    packet_builder("YAWMOVE", 'f', steer2_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 250) == 82)
  {
    // ACCEL_X - x acceleration
    packet_builder("ACCEL_X", 'd', x_accel);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 250) == 124)
  {
    // ACCEL_Y - x acceleration
    packet_builder("ACCEL_Y", 'd', y_accel);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 250) == 132)
  {
    // ACCEL_Z - x acceleration
    packet_builder("ACCEL_Z", 'd', z_accel);
    Serial4.write(telem_packet, 24);
  }

#ifndef SEND_RPY
  if ((comm_time % 250) == 174)
  {
    // ROLLVAL - IMU roll value
    packet_builder("ROLLVAL", 'd', roll);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 250) == 182)
  {
    // PIT_VAL - IMU pitch value
    packet_builder("PIT_VAL", 'd', pitch);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 250) == 190)
  {
    // YAW_VAL - IMU yaw value
    packet_builder("YAW_VAL", 'd', yaw);
    Serial4.write(telem_packet, 24);
  }
#endif

  // 1Hz data (1000ms)

  if ((comm_time % 1000) == 224)
  {
    // PIDVLRP - PID roll proportional value
    packet_builder("PIDVLRP", 'f', kpr_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 1000) == 232)
  {
    // PIDVLRI - PID roll integral value
    packet_builder("PIDVLRI", 'f', kir_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 1000) == 474)
  {
    // PIDVLRD - PID roll derivative value
    packet_builder("PIDVLRD", 'f', kdr_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 1000) == 482)
  {
    // PIDVLPP - PID pitch proportional value
    packet_builder("PIDVLPP", 'f', kpp_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 1000) == 724)
  {
    // PIDVLPI - PID pitch integral value
    packet_builder("PIDVLPI", 'f', kip_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 1000) == 732)
  {
    // PIDVLPD - PID pitch derivative value
    packet_builder("PIDVLPD", 'f', kdp_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 1000) == 974)
  {
    // MODEVAL - mode that Kevin is in
    packet_builder("MODEVAL", 'i', mode_commanded);
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 1000) == 982)
  {
    // BALMODE - type of balance being used
    packet_builder("BALMODE", 'i', mode_commanded); // placeholder
    Serial4.write(telem_packet, 24);
  }
  else if ((comm_time % 1000) == 990)
  {
    // KEVTIME - time onboard Kevin (milliseconds)
    packet_builder("KEVTIME", 'i', comm_time); // placeholder
    Serial4.write(telem_packet, 24);
  }
}

void comms_receive()
{

  /*
  1. if 1 byte available, parse it
    if character parses as "<", start recording, go to 2
  2. record checksum
  3. capture time - record time (int)
    if 4 bytes in buffer, read/store as int
    save in working var for time stamp, go to 3
  4. capture mnemonic - capture 7 more characters
    if 7 bytes in buffer, read/store as 7 char string
    save in working var for mnemonic
  5. read data type: bool, int, float, double
    determine how to read the data
  6-9. capture value - record characters based on data type
    read # or bytes based on specified datatype
  10. return result
    save working vars to complete packet var
    set new data flag to true
  */

  //  1. if 1 byte available, parse it
  if (comm_receive_step == 1)
  {
    if (Serial4.available() > 0)
    {

      char start_byte;
      start_byte = Serial4.read();
      if (start_byte == '<')
      {
#ifdef SERIAL_MIRROR
        Serial.print("\t");
        Serial.print(start_byte);
#endif
        comm_receive_step++;
      }
      comm_results.checksum2 = 0;
    }
  }
  // 2. parse checksum
  if (comm_receive_step == 2)
  {
    if (Serial4.available() >= 2)
    {

      byte checksum_buffer[2];
      for (int i = 0; i < 2; i++)
      {

        checksum_buffer[i] = Serial4.read();
      }
      memcpy(&comm_results.checksum1, checksum_buffer, sizeof(comm_results.checksum1));
#ifdef SERIAL_MIRROR_CHECKSUM
      Serial.print(comm_results.checksum1);
#endif
      comm_receive_step++;
    }
  }

  // 3. parse time
  if (comm_receive_step == 3)
  {
    if (Serial4.available() >= 4)
    {
      byte time_buffer[4];
      for (int i = 0; i < 4; i++)
      {

        time_buffer[i] = Serial4.read();
        comm_results.checksum2 = comm_results.checksum2 + time_buffer[i];
      }

      memcpy(&time_stamp, time_buffer, sizeof(time_stamp));
      comm_receive_step++;

#ifdef SERIAL_MIRROR_TIME
      Serial.print(time_stamp);
#endif
    }
  }
  // 4. parse mnemonic - capture 7 characters
  if (comm_receive_step == 4)
  {
    if (Serial4.available() >= 7)
    {
      for (int i = 0; i < 7; i++)
      {
        mnemonic[i] = (byte)Serial4.read();
        comm_results.checksum2 = comm_results.checksum2 + mnemonic[i];
      }
#ifdef SERIAL_MIRROR_MNEMONIC
      Serial.print(mnemonic);
#endif
      comm_receive_step++;
    }
  }

  // 5. capture datatype - bool, int, float, double
  if (comm_receive_step == 5)
  {
    if (Serial4.available() >= 1)
    {

      data_byte = Serial4.read();
      comm_results.checksum2 = comm_results.checksum2 + data_byte;

#ifdef SERIAL_MIRROR_TYPE
      Serial.print(data_byte);
#endif

      if (data_byte == 'b')
      {
        comm_receive_step = 6;
      }
      if (data_byte == 'i')
      {
        comm_receive_step = 7;
      }
      if (data_byte == 'f')
      {
        comm_receive_step = 8;
      }
      if (data_byte == 'd')
      {
        comm_receive_step = 9;
      }
    }
  }

  // 6-9. capture data - 8 bytes, regardless of type
  // bool
  if (comm_receive_step == 6)
  {
    if (Serial4.available() >= 8)
    {

      byte bool_buffer[2];
      for (int i = 0; i < 2; i++)
      {
        bool_buffer[i] = Serial4.read();
        comm_results.checksum2 = comm_results.checksum2 + bool_buffer[i];
      }

      for (int i = 0; i < 6; i++)
      {
        comm_results.checksum2 = comm_results.checksum2 + Serial4.read();
      }
      memcpy(&comm_data.b, bool_buffer, sizeof(comm_data.b));

      comm_results.time = time_stamp;
      comm_results.mnemonic = mnemonic;
      comm_results.data.b = comm_data.b;
      comm_receive_step = 10;
#ifdef SERIAL_MIRROR_DATA
      Serial.print(comm_data.b);
#endif
    }
  }
  // int
  if (comm_receive_step == 7)
  {
    if (Serial4.available() >= 8)
    {

      byte int_buffer[4];
      for (int i = 0; i < 4; i++)
      {

        int_buffer[i] = Serial4.read();
        comm_results.checksum2 = comm_results.checksum2 + int_buffer[i];

      }

      for (int i = 0; i < 4; i++)
      {
        comm_results.checksum2 = comm_results.checksum2 + Serial4.read();
      }

      memcpy(&comm_data.i, int_buffer, sizeof(comm_data.i));
     
      comm_results.time = time_stamp;
      comm_results.mnemonic = mnemonic;
      comm_results.data.i = comm_data.i;
      comm_receive_step = 10;
#ifdef SERIAL_MIRROR_DATA
      Serial.print(comm_data.i);
#endif
    }
  }
  // float
  if (comm_receive_step == 8)
  {
    if (Serial4.available() >= 8)
    {
      byte float_buffer[4];
      for (int i = 0; i < 4; i++)
      {
        float_buffer[i] = Serial4.read();
        comm_results.checksum2 = comm_results.checksum2 + float_buffer[i];
      }

      for (int i = 0; i < 4; i++)
      {
        comm_results.checksum2 = comm_results.checksum2 + Serial4.read();
      }

      memcpy(&comm_data.f, float_buffer, sizeof(comm_data.f));

      comm_results.time = time_stamp;
      comm_results.mnemonic = mnemonic;
      comm_results.data.f = comm_data.f;
      comm_receive_step = 10;
#ifdef SERIAL_MIRROR_DATA
      Serial.print(comm_data.f);
#endif
    }
  }
  // double
  if (comm_receive_step == 9)
  {

    if (Serial4.available() >= 8)
    {

      byte double_buffer[8];
      for (int i = 0; i < 8; i++)
      {

        double_buffer[i] = Serial4.read();
        comm_results.checksum2 = comm_results.checksum2 + double_buffer[i];
      }

      memcpy(&comm_data.d, double_buffer, sizeof(comm_data.d));

      comm_results.time = time_stamp;
      comm_results.mnemonic = mnemonic;
      comm_results.data.d = comm_data.d;
      comm_receive_step = 10;
#ifdef SERIAL_MIRROR_DATA
      Serial.print(comm_data.d);
#endif
    }
  }
  // 10. validate message
  if (comm_receive_step == 10)
  {
    if (Serial4.available() >= 1)
    {
      char end_byte = Serial4.read(); // check if E?
      comm_receive_step = 1;

      if ((int)comm_results.checksum1 == (int)comm_results.checksum2)
      {
        new_data = 1;
      }
      else
      {
        new_data = 0;
#ifdef CHECKSUM_FAIL
        Serial.println("bad data");
#endif
      }

#ifdef SERIAL_MIRROR_VALID
      Serial.print(comm_results.time);
      Serial.print(comm_results.mnemonic);
      Serial.print(data_byte);
      Serial.print("\t");
      Serial.print("\t");
      Serial.print("\t");

      Serial.print(comm_results.data.f);
      Serial.print("\t");
      Serial.print(comm_data.f);
      Serial.print("\t");
      Serial.print(comm_results.data.d);
      Serial.print("\t");
      Serial.println(comm_data.d);

#endif
#ifdef SERIAL_MIRROR
      Serial.print(end_byte);
      Serial.print("\t");
      Serial.print(comm_results.checksum1);
      Serial.print("\t");
      Serial.println(comm_results.checksum2);

#endif
      comm_results.checksum2 = 0;
    }
  }
}

void comms_interpreter()
{
  if (new_data == 1)
  {
#ifdef COMM_ROLE_VEHICLE
    if (comm_results.mnemonic == "FWDMOVE")
    {
      x_dist = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == "SIDMOVE")
    {
      //steer2 = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == "YAWMOVE")
    {
      steer2 = comm_results.data.f;
      new_data = 0;
    }

    if (comm_results.mnemonic == "MODEVAL")
    {
      mode = comm_results.data.i;
      new_data = 0;
    }

    if (comm_results.mnemonic == "PIDVLRP")
    {
      kpr = comm_results.data.f;
      rollPID.SetTunings(kpr, kir, kdr);
      new_data = 0;
    }

    if (comm_results.mnemonic == "PIDVLRI")
    {
      kir = comm_results.data.f;
      rollPID.SetTunings(kpr, kir, kdr);
      new_data = 0;
    }

    if (comm_results.mnemonic == "PIDVLRD")
    {
      kdr = comm_results.data.f;
      rollPID.SetTunings(kpr, kir, kdr);
      new_data = 0;
    }

    if (comm_results.mnemonic == "PIDVLPP")
    {
      kpp = comm_results.data.f;
      pitchPID.SetTunings(kpp, kip, kdp);
      new_data = 0;
    }

    if (comm_results.mnemonic == "PIDVLPI")
    {
      kip = comm_results.data.f;
      pitchPID.SetTunings(kpp, kip, kdp);
      new_data = 0;
    }

    if (comm_results.mnemonic == "PIDVLPD")
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

    else if (comm_results.mnemonic == "PIT_VAL")
    {
      pitch = comm_results.data.d;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "YAW_VAL")
    {
      yaw = comm_results.data.d;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "ACCEL_X")
    {
      x_accel = comm_results.data.d;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "ACCEL_Y")
    {
      y_accel = comm_results.data.d;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "ACCEL_Z")
    {
      z_accel = comm_results.data.d;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "STEPSTE")
    {
      state = comm_results.data.i;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "H1A_TAR")
    {
      leg1.hip = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "S1A_TAR")
    {
      leg1.shoulder = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "K1A_TAR")
    {
      leg1.knee = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "L1X_POS")
    {
      x1.pos = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "L1Y_POS")
    {
      yy1.pos = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "L1Z_POS")
    {
      z1.pos = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "PIDVLRP")
    {
      kpr = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "PIDVLRI")
    {
      kir = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "PIDVLRD")
    {
      kdr = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "PIDVLPP")
    {
      kpp = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "PIDVLPI")
    {
      kip = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "PIDVLPD")
    {
      kdp = comm_results.data.f;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "BALMODE")
    {
      // bal_mode = comm_results.data.i;
      new_data = 0;
    }

    else if (comm_results.mnemonic == "KEVTIME")
    {
      // time = comm_results.data.i;
      new_data = 0;
    }
    else
    {
      new_data = 0;
      return;
    }
#ifdef COMM_SHOW_INTERPRETER
    Serial.print(comm_results.time);
    Serial.print("\t");
    Serial.print(comm_results.mnemonic);
    Serial.print("\t");
    Serial.println(comm_results.data.d);
#endif
#endif
  }
}

#endif