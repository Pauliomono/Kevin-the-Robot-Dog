#include <remote_control.h>

// grab input function (for remote control)
void remoteIO()
{
#ifdef RC_MODE_INT
  get_input_int();
#endif
#ifdef RC_MODE_STRING
  assemble_input();
#endif

  send_telemetry();
}

void get_input_int()
{
  if (Serial4.available() > 0)
  {

#ifdef REMOTE_DATA_PRINT_RAW
    Serial.print(Serial4.available());
    Serial.print("\t");
#endif
    // packet = Serial4.read();
    packet2 = Serial4.parseInt();

#ifdef REMOTE_DATA_PRINT_RAW
    Serial.print(packet2);
    Serial.print("\t");
    // Serial.print(packet);
    // Serial.print("\t");
#endif
    interpret_input2(packet2);
  }
  else
  {
    /*
#ifdef REMOTE_DATA_PRINT_RAW
    Serial.print(Serial4.available());
#endif
  */
  }
}

void assemble_input()
{
  String packet;
  
  if (Serial4.available())
  {
    //Serial.print(Serial4.available());
    //Serial.print("\t");
    packet = Serial4.read();
    if (packet == "n")
    {
      //Serial.println(datapacket);
      data_recieved = true;
      interpret_input_string();
      datapacket = "";
    }
    else if (packet == "x")
    {
      datapacket = "";
    }
    else
    {
      datapacket += packet;
      //Serial.println(datapacket);
    }
  }
}

void send_telemetry()
{
  int sendmode = 3000 + mode;

  Serial4.println(sendmode);
  Serial4.println("\t");
}