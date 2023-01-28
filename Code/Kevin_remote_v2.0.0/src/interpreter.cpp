#include <interpreter.h>

void interpret_input_string()
{
  String xstring = "";
  String ystring = "";
  String bt1 = String(datapacket[8]);
  String bt2 = String(datapacket[9]);
  String bt3 = String(datapacket[10]);
  
  xstring += datapacket[1];
  xstring += datapacket[2];
  xstring += datapacket[3];
  ystring += datapacket[5];
  ystring += datapacket[6];
  ystring += datapacket[7];

  xxx = xstring.toInt();
  yyy = ystring.toInt();

  if (datapacket[0] == '0')
  {
    xxx = -1 * xxx;
  }
  if (datapacket[4] == '0')
  {
    yyy = -1 * yyy;
  }

  if (mode == 2 || mode == 3)
  {
    
  }

//modes update
  mode = bt1.toInt();
  tune_mode = bt3.toInt();

}
