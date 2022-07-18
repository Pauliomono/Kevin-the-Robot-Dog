#include <balance_controller.h>

void balance(){
  //add rpy offsets - will be implemented into balance control
  /*
  pitch1 = l_pitch*sin(pitch_angle1/180*PI);
  roll1 = l_roll*sin(roll_angle1/180*PI);
  yaw2 = -l_pitch*sin(yaw1/180*PI);
  */
  pitch1 = leg_height*sin(pitch_angle1/180*PI);
  roll1 = leg_height*sin(roll_angle1/180*PI);

  x1.pos_f = x1.pos_f - pitch1;
  x2.pos_f = x2.pos_f - pitch1;
  x3.pos_f = x3.pos_f + pitch1;
  x4.pos_f = x4.pos_f + pitch1;

  yy1.pos_f = yy1.pos_f + roll1;
  yy2.pos_f = yy2.pos_f - roll1;
  yy3.pos_f = yy3.pos_f + roll1;
  yy4.pos_f = yy4.pos_f + roll1;
  /*
  z1 = z1 - pitch1 - roll1;
  z2 = z2 - pitch1 + roll1;
  z3 = z3 + pitch1 + roll1;
  z4 = z4 + pitch1 - roll1;
  */
  
  pitch2 = sin(pitch_angle2/180*PI);

  /*
  if((state == 1)||(state == 2)||(state == 3)){
  
  //yy1 = yy1 - z1*roll2;
  yy2 = yy2 + z2*roll_angle2;
  //yy3 = yy3 + z3*roll2;
  yy4 = yy4 - z4*roll_angle2;

  yy2 = constrain(yy2, -100, 10);
  yy4 = constrain(yy4, -100, 10);

  }

  if((state == 4)||(state == 5)||(state == 6)){
  
  yy1 = yy1 - z1*roll_angle2;
  //yy2 = yy2 + z2*roll_angle2;
  yy3 = yy3 - z3*roll_angle2;
  //yy4 = yy4 - z4*roll_angle2;

  yy1 = constrain(yy1, -100, 10);
  yy3 = constrain(yy3, -10, 100);

  }
  */
}