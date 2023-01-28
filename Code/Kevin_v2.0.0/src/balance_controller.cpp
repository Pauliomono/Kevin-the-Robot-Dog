#include <balance_controller.h>

void balance()
{
  // add rpy offsets - will be implemented into balance control
  /*
  pitch1 = l_pitch*sin(pitch_angle1/180*PI);
  roll1 = l_roll*sin(roll_angle1/180*PI);
  yaw2 = -l_pitch*sin(yaw1/180*PI);
  */
#ifdef PID_BALANCE_ENABLE
  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();
#endif

  pitch1 = leg_height * sin(pitch_angle1 / 180 * PI);
  roll1 = leg_height * sin(roll_angle1 / 180 * PI);

  // check to only modify feet touching the ground
  if (mode == 1 || mode == 3)
  {
    if (state == 2 || state == 3 || state == 4 || state == 5)
    {
      x1.pos_f = x1.pos_f - pitch1;
      x3.pos_f = x3.pos_f + pitch1;
      yy1.pos_f = yy1.pos_f + roll1;
      yy3.pos_f = yy3.pos_f + roll1;
    }
    else if (state == 6 || state == 7 || state == 8 || state == 1)
    {
      x2.pos_f = x2.pos_f - pitch1;
      x4.pos_f = x4.pos_f + pitch1;
      yy2.pos_f = yy2.pos_f + roll1;
      yy4.pos_f = yy4.pos_f + roll1;
    }
  }

  if (mode == 0 || mode == 2)
  {
    x1.pos_f = x1.pos_f - pitch1;
    x2.pos_f = x2.pos_f - pitch1;
    x3.pos_f = x3.pos_f + pitch1;
    x4.pos_f = x4.pos_f + pitch1;

    yy1.pos_f = yy1.pos_f + roll1;
    yy2.pos_f = yy2.pos_f + roll1;
    yy3.pos_f = yy3.pos_f + roll1;
    yy4.pos_f = yy4.pos_f + roll1;
  }
  /*
  z1 = z1 - pitch1 - roll1;
  z2 = z2 - pitch1 + roll1;
  z3 = z3 + pitch1 + roll1;
  z4 = z4 + pitch1 - roll1;
  */

  // pitch2 = sin(pitch_angle2/180*PI);

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

int input_shape_setup(double target_angle, double feedback_angle, int n_sample_count)
{
  /*--------------------------------------------------------------------------------------
  get the roots/time of roots

  This is accomplished with a trigger range +/- .25 degrees from target angle
  The root is approximated by averaging the times for values within the trigger range
  --------------------------------------------------------------------------------------*/
  if (step == 0) // initial setup
  {
    trigger_state = false;
    root_time_counter = 1;
    double delta = .5;
    low_trigger = target_angle - delta;
    high_trigger = target_angle + delta;
    peak = target_angle;
    trough = target_angle;
    mpeak = 0;
    mtrough = 0;
    step = 1;
    roots_delta = 0;
    t_sin_init = millis();
  }
  //-----------------------------------------------------------------------------------------
  if (step == 1) // 1 get peak/peak time
  {

    if (feedback_angle > peak)
    {
      peak = feedback_angle;
      peak_time = millis() - t_sin_init;
    }
    if (((peak - feedback_angle) > (peak - high_trigger) / 2) && (peak > high_trigger))
    {
      step = 2;
    }
  }
  //-----------------------------------------------------------------------------------------
  if (step == 2) // 2 get high->low root
  {
    if (!trigger_state)
    {
      // above target_angle
      if (feedback_angle <= high_trigger)
      {
        trigger_state = true;
        root_time = millis() - t_sin_init;
      }
    }
    else if (trigger_state)
    {
      if (feedback_angle < low_trigger)
      {
        trigger_state = false;
        root_time = (double)root_time / root_time_counter;
        root_time_counter = 1;
        roots_delta = root_time;
        root_time_1 = root_time; // store root_time as one of the roots
        if (n_sample_count == 0)
        {
          phase_time = root_time;
        }

        step = 3;
      }
      else
      {
        root_time += millis() - t_sin_init;
        root_time_counter++;
      }
    }
  }
  //-----------------------------------------------------------------------------------------
  if (step == 3) // 3 get trough
  {
    if (feedback_angle < trough)
    {
      trough = feedback_angle;
      trough_time = millis() - t_sin_init;
    }
    if (((feedback_angle - trough) > (low_trigger - trough) / 2) && (trough < low_trigger))
    {
      step = 4;
    }
  }
  //-----------------------------------------------------------------------------------------
  if (step == 4) // 4 get low to high root
  {
    if (!trigger_state)
    {

      // below target_angle

      if (feedback_angle >= low_trigger)
      {
        trigger_state = true;
        root_time = millis() - t_sin_init;
      }
    }
    else if (trigger_state)
    {
      // increasing

      if (feedback_angle > high_trigger)
      {
        trigger_state = false;
        root_time = (double)root_time / root_time_counter;
        root_time_counter = 1;
        root_time_2 = root_time; // store root_time as one of the roots
        step = 5;
      }
      else
      {
        root_time += millis() - t_sin_init;
        root_time_counter++;
      }
    }
  }
  //-----------------------------------------------------------------------------------------
  if (step == 5) // compute results
  {
    // frequency = (double)1000 / (((root_time_1 - peak_time) + (trough_time - root_time_1) + (root_time_2 - trough_time)) / 3 * 4);
    n_sample_count++;
    frequency = (double)1000 * (n_sample_count + 1) / roots_delta;
    mpeak += peak;
    mtrough += trough;
    amplitude = (mpeak - mtrough) / (2 * n_sample_count);

    peak = target_angle;
    trough = target_angle;
    step = 1;
  }
  return n_sample_count;
}

double input_shaper(double input, double f, double A)
{
  /*--------------------------------------------------------------------------------------
   take commanded signal and shape it to damp oscillations

   --------------------------------------------------------------------------------------*/

  double output;

  //output = (double)input + .4 * A * sin(2 * PI * f * (millis() - t_sin_init - phase_time) / 1000 + PI);

  if (remainder(millis(), 12*1000/f) <= 6*1000/f)
  {
    output = (double)input + .5 * A * sin(2 * PI * f * (millis() - t_sin_init - phase_time) / 1000 + PI);
  }
  else
  {
    output = (double)input + .25 * A * sin(2 * PI * f * (millis() - t_sin_init - phase_time) / 1000);
  }

  return output;

  // to add: potential range where input shaping is active: might only be needed around 90 angle
}