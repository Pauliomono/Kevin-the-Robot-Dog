#include <interpreter.h>

void interpret_input2(int packet)
{
  int datatype = packet / 1000;
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

  if (modetoggle == 1)
  {
    mode++;
  }

  if (mode == 0 || mode == 2)
  {
    state_time = 100;
    interpolation_interval = state_time / n_interps;
  }

  if (mode == 1)
  {
    state_time = 50;
    interpolation_interval = state_time / n_interps;
  }

  if (mode == 3)
  {
    state_time = 50;
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
  steer1 = map(yyy, -98, 99, -motion_limit / 2, motion_limit / 2);
  steer2 = map(yyy, -98, 99, motion_limit / 2, -motion_limit / 2);

#ifdef REMOTE_DATA_PRINT
  Serial.print(" x_dist = ");
  Serial.print(x_dist);
  Serial.print(" steer1 = ");
  Serial.print(steer1);
  Serial.print(" mode = ");
  Serial.println(mode);
#endif
}

void interpret_input_string()
{
  int xxx;
  int yyy;
  String xstring = "";
  String ystring = "";
  String bt1 = datapacket[8];
  String bt2 = datapacket[9];
  String bt3 = datapacket[10];
  data_recieved = 0;

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
    xxx = (int)xxx / 90;
    yyy = (int)yyy / 90;

    if (xxx != xxx_old)
    {
      x_tune = xxx;
    }
    else {
      x_tune = 0;
    }
    if (yyy != yyy_old)
    {
      y_tune = yyy;
    }
    else {
      y_tune = 0;
    }
    xxx_old = xxx;
    yyy_old = yyy;
  }

//toggle buttons update
  int mode_button = bt1.toInt();
  int precision_button = bt2.toInt();
  int tune_button = bt3.toInt();

  if (mode_button == 1)
  {
    mode++;
    tune_precision = 0;
    tune_mode = 0;
  }

  if (precision_button == 1)
  {
    tune_precision++;
  }

  if (tune_button == 1)
  {
    tune_mode++;
  }

//mode interpreter
  if (mode == 0)
  {
    state_time = 100;
    interpolation_interval = state_time / n_interps;
  }

  if (mode == 1)
  {
    state_time = 50;
    interpolation_interval = state_time / n_interps;
  }

  if (mode == 2)
  {
    
    state_time = 100;
    interpolation_interval = state_time / n_interps;
  }

  if (mode == 3)
  {
    state_time = 100;
    interpolation_interval = state_time / n_interps;
  }

// toggle variable resets
  if (mode > NMODES)
  {
    mode = 0;
  }

  if (tune_precision > 2){
    tune_precision = 0;
  }

  if (tune_mode > 2){
    tune_mode = 0;
  }

#ifdef REMOTE_DATA_PRINT
  Serial.print(" x = ");
  Serial.print(xxx);
  Serial.print(" y = ");
  Serial.print(yyy);
#endif

//remote actions

  if (mode == 0 || mode == 1)
  {
    x_dist = map(xxx, -98, 99, -motion_limit, motion_limit);
    // steer1 = map(yyy, -98, 99, -motion_limit, motion_limit);
    steer2 = map(yyy, -98, 99, motion_limit, -motion_limit);
  }

  if (mode == 2 || mode == 3){
    if (tune_precision == 1){
      x_tune /= 10;
      y_tune /= 10;
    }
    if (tune_precision == 2){
      x_tune /= 100;
      y_tune /= 100;
    }

    if(tune_mode == 0){
      kpp += x_tune;
      kpr += y_tune;
    }
    if(tune_mode == 1){
      kip += x_tune;
      kir += y_tune;
    }
    if(tune_mode == 2){
      kdp += x_tune;
      kdr += y_tune;
    }

    #ifdef PID_BALANCE_ENABLE
    rollPID.SetTunings(kpr,kir,kdr);
    pitchPID.SetTunings(kpp,kip,kdp);
    #endif
  }

#ifdef REMOTE_DATA_PRINT
  Serial.print(" x_dist = ");
  Serial.print(x_dist);
  Serial.print(" steer1 = ");
  Serial.print(steer2);
  Serial.print(" mode = ");
  Serial.println(mode);
#endif
}
