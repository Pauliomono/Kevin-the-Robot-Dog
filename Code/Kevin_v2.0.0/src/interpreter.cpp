#include <interpreter.h>

void interpret_input2(int packet)
{
  int datatype = packet/1000;
  int modetoggle = 0;
  
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

  if(modetoggle == 1){
    mode ++;
  }

  if (mode == 0 || mode == 2)
  {
    state_time = 100;
    interpolation_interval = state_time / n_interps;
  }

  if (mode == 1)
  {
    state_time = 50;
    interpolation_interval = state_time /n_interps;
  }

  if (mode == 3)
  {
    state_time = 1000;
    interpolation_interval = state_time / n_interps;
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
#endif
  

  x_dist = map(xxx, -98, 99, -motion_limit, motion_limit);
  steer1 = map(yyy, -98, 99, -motion_limit/2, motion_limit/2);
  steer2 = map(yyy, -98, 99, motion_limit/2, -motion_limit/2);

#ifdef REMOTE_DATA_PRINT
  Serial.print(" x_dist = ");
  Serial.print(x_dist);
  Serial.print(" steer1 = ");
  Serial.print(steer1);
  Serial.print(" mode = ");
  Serial.println(mode);
#endif

  
   
}

void interpret_input_string(){
  int xxx;
  int yyy;
  String xstring = "";
  String ystring = "";
  String bt1 = datapacket[8];
  data_recieved = 0;
  
  xstring += datapacket[1];
  xstring += datapacket[2];
  xstring += datapacket[3];
  ystring += datapacket[5];
  ystring += datapacket[6];
  ystring += datapacket[7];

  xxx = xstring.toInt();
  yyy = ystring.toInt();
  if (datapacket[0] == '0'){
    xxx = -1*xxx;
  }
  if (datapacket[4] == '0'){
    yyy = -1*yyy;
  }

  int mode_button = bt1.toInt();

  if(mode_button == 1){
    mode ++;
  }

  if (mode == 0 || mode == 2)
  {
    state_time = 100;
    interpolation_interval = state_time / n_interps;
  }

  if (mode == 1)
  {
    state_time = 50;
    interpolation_interval = state_time /n_interps;
  }

  if (mode == 3)
  {
    state_time = 1000;
    interpolation_interval = state_time / n_interps;
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
#endif
  
  x_dist = map(xxx, -98, 99, -motion_limit, motion_limit);
  steer1 = map(yyy, -98, 99, -motion_limit, motion_limit);
  steer2 = map(yyy, -98, 99, motion_limit, -motion_limit);

#ifdef REMOTE_DATA_PRINT
  Serial.print(" x_dist = ");
  Serial.print(x_dist);
  Serial.print(" steer1 = ");
  Serial.print(steer1);
  Serial.print(" mode = ");
  Serial.println(mode);
#endif

}


