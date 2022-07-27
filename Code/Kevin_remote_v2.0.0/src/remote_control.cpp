#include <remote_control.h>

void remoteIO(int X, int Y, bool b1, bool b2, bool b3, bool b4)
{
  get_telemetry();
  send_inputs_string(X, Y, b1, b2, b3, b4);
}

void send_inputs_int(int X, int Y, bool b1, bool b2, bool b3, bool b4)
{

  if (X >= 0)
  {
    X += 1100;
  }
  else
  {
    X = 1000 + abs(X);
  }

  if (Y >= 0)
  {
    Y += 2100;
  }
  else
  {
    Y = 2000 + abs(Y);
  }

  int buttons = 3000;
  if (b1)
  {
    buttons += 1;
  }

  SerialBT.println(X);
  SerialBT.println("\t");
  SerialBT.println(Y);
  SerialBT.println("\t");
  SerialBT.println(buttons);
  SerialBT.println("\t");
  Serial.println(X);
  Serial.println(Y);
  Serial.println(buttons);
}

void send_inputs_string(int X, int Y, bool b1, bool b2, bool b3, bool b4)
{
  String datapacket;
  datapacket = "x";
  datapacket += num2str(X);
  datapacket += num2str(Y);
  datapacket += bool2str(b1);
  datapacket += bool2str(b2);
  datapacket += bool2str(b3);
  datapacket += bool2str(b4);
  datapacket += "n";
  
  Serial.println(datapacket);
  SerialBT.println(datapacket);
}

String num2str(int X){
  String packet;
  if (X >=0){
    packet = "1";
  }
  else{
    packet = "0";
  }
  if (abs(X) < 10){
    packet += "00";
  }
  else if (abs(X) < 100){
    packet += "0";
  }

  packet += abs(X);

   return(packet);
}

String bool2str(bool X)
{
  String packet;
  if (X)
  {
    packet = "1";
  }
  else
  {
    packet = "0";
  }

  return (packet);
}

void get_telemetry()
{

  if (SerialBT.available() > 0)
  {

    int packet = SerialBT.parseInt();
    int datatype = packet / 1000;

    if (datatype == 3)
    {
      mode = packet - 3000;
    }
  }
}



