#include <interpreter.h>

void interpret_input2(int packet)
{
  int datatype = packet/1000;
  bool modetoggle = 0;
  
  if (datatype == 1)
  {
    xxx = packet - 1000;
    if (xxx / 100 == 1)
    {
      xxx -= 100;
    }
    else
    {
      xxx *= -1;
    }
  }

  if (datatype == 2)
  {
    yyy = packet - 2000;
    if (yyy / 100 == 1)
    {
      yyy -= 100;
    }
    else
    {
      yyy *= -1;
    }
  }

  if (datatype == 3)
  {
    modetoggle = packet - 3000;
  }

  if(modetoggle){
    mode ++;
  }

  if (mode == 0 || mode == 2)
  {
    state_time = 1000000;
  }

  if (mode == 1)
  {
    state_time = 100000;
  }

  if (mode == 3)
  {
    state_time = 1000000;
  }

  if (mode > NMODES)
  {
    mode = 0;
  }

#ifdef REMOTE_DATA_PRINT
  Serial.print(" x = ");
  Serial.print(xxx);
  Serial.print(" y = ");
  Serial.print(yyy);
  Serial.print(" mode = ");
  Serial.println(mode);
#endif
  

  x_dist = map(xxx, -98, 99, -motion_limit, motion_limit);
  steer1 = map(yyy, -98, 99, -motion_limit, motion_limit);
  steer2 = map(yyy, -98, 99, motion_limit, -motion_limit);
  
   
}
