#include <servos.h>

void write_servos()
{
#ifdef INPUT_SHAPING
  leg3.hip = input_shaper(leg3.hip, frequency, amplitude);
#endif

#ifdef SERVO_PWM_INTERNAL
  hip1.write(85 - leg1.hip);
  shoulder1.write(96 + leg1.shoulder);
  knee1.write(180 + 2 - leg1.knee);
  hip2.write(95 + leg2.hip);
  shoulder2.write(90 - leg2.shoulder);
  knee2.write(3 + leg2.knee);
  hip3.write(93 + leg3.hip);
  shoulder3.write(96 + leg3.shoulder);
  knee3.write(180 + 5 - leg3.knee);
  hip4.write(91 + leg4.hip);
  shoulder4.write(95 - leg4.shoulder);
  knee4.write(10 + leg4.knee);
#endif

#ifdef SERVO_PWM_EXTERNAL
  int hip1m = deg2micros(85 - leg1.hip);
  int shoulder1m = deg2micros(96 + leg1.shoulder);
  int knee1m = deg2micros(180 + 2 - leg1.knee);
  int hip2m = deg2micros(95 + leg2.hip);
  int shoulder2m = deg2micros(90 - leg2.shoulder);
  int knee2m = deg2micros(3 + leg2.knee);
  // int hip3m = deg2micros(93 + leg3.hip);

  int hip3m;
  if (remainder(millis(), 2000) < 75)
  {
    hip3m = deg2micros(93 + leg3.hip);
  }
  else
  {
    hip3m = 0;
  }
  // int hip3m = deg2micros(93 + 0);
  int shoulder3m = deg2micros(96 + leg3.shoulder);
  int knee3m = deg2micros(180 + 5 - leg3.knee);
  int hip4m = deg2micros(91 + leg4.hip);
  int shoulder4m = deg2micros(95 - leg4.shoulder);
  int knee4m = deg2micros(10 + leg4.knee);

  pwm.writeMicroseconds(0, hip1m);
  pwm.writeMicroseconds(1, shoulder1m);
  pwm.writeMicroseconds(2, knee1m);
  pwm.writeMicroseconds(3, hip2m);
  pwm.writeMicroseconds(4, shoulder2m);
  pwm.writeMicroseconds(5, knee2m);
  pwm.writeMicroseconds(10, hip3m); // servodeadzone(1600, 1496, 24)
  pwm.writeMicroseconds(11, shoulder3m);
  pwm.writeMicroseconds(12, knee3m);
  pwm.writeMicroseconds(13, hip4m);
  pwm.writeMicroseconds(14, shoulder4m);
  pwm.writeMicroseconds(15, knee4m);

  pwm.writeMicroseconds(7, 1500);
#endif
}

// UNUSED FUNCTION
// adding deadzones did not work, oscillation problem is not caused by potentiometers:
int servodeadzone(int mils, int dead_center, int dead_width)
{
  int low = dead_center - dead_width;
  int high = dead_center + dead_width;

  if (mils < dead_center && mils > low)
  {
    mils = low;
  }

  if (mils > dead_center && mils < high)
  {
    mils = high;
  }

  return mils;
}

int deg2micros(float degrees)
{
  int mics;
  mics = (float)544 + degrees * 464 / 45;
  return mics;
  // map(degrees, 0, 180, 544, 2400); 1856/180
}