#include <kinematics.h>

// inverse kinematics function - calculates leg angles from xyz foot coords
struct angles IK_final(float rx, float ry, float rz)
{
  struct angles theta;
  float r = sqrt(pow(rz, 2) + pow(rx, 2));

  theta.hip = 180 / PI * atan2(ry, rz);
  theta.knee = 180 / PI * (acos((2 * pow(L, 2) - pow(r, 2)) / (2 * L * L)));
  theta.shoulder = 180 / PI * (acos((pow(r, 2)) / (2 * r * L)) - atan2(rx, rz));

  return (theta);
}

// interpolation function - gets angles between current and final angle
struct angles IK_interpolate(struct angles leg_final, struct angles leg, float t0, float tf, float t)
{
  struct angles theta;

  // interpolate angles based on time left for state
  theta.hip = interpolate2(leg_final.hip, leg.hip, t0, tf, t);
  theta.shoulder = interpolate2(leg_final.shoulder, leg.shoulder, t0, tf, t);
  theta.knee = interpolate2(leg_final.knee, leg.knee, t0, tf, t);

  return (theta);
}

// interpolation function - gets angles between current and final angle
void IK_interpolate2(float t0, float tf, float t)
{

  // interpolate next cordinate position

  x1.pos = interpolate1(x1.pos_f, x1.pos, x1.vel_f, x1.accel_f, t0, tf, t);
  yy1.pos = interpolate1(yy1.pos_f, yy1.pos, yy1.vel_f, yy1.accel_f, t0, tf, t);
  z1.pos = interpolate1(z1.pos_f, z1.pos, z1.vel_f, z1.accel_f, t0, tf, t);
  x2.pos = interpolate1(x2.pos_f, x2.pos, x2.vel_f, x2.accel_f, t0, tf, t);
  yy2.pos = interpolate1(yy2.pos_f, yy2.pos, yy2.vel_f, yy2.accel_f, t0, tf, t);
  z2.pos = interpolate1(z2.pos_f, z2.pos, z2.vel_f, z2.accel_f, t0, tf, t);
  x3.pos = interpolate1(x3.pos_f, x3.pos, x3.vel_f, x3.accel_f, t0, tf, t);
  yy3.pos = interpolate1(yy3.pos_f, yy3.pos, yy3.vel_f, yy3.accel_f, t0, tf, t);
  z3.pos = interpolate1(z3.pos_f, z3.pos, z3.vel_f, z3.accel_f, t0, tf, t);
  x4.pos = interpolate1(x4.pos_f, x4.pos, x4.vel_f, x4.accel_f, t0, tf, t);
  yy4.pos = interpolate1(yy4.pos_f, yy4.pos, yy4.vel_f, yy4.accel_f, t0, tf, t);
  z4.pos = interpolate1(z4.pos_f, z4.pos, z4.vel_f, z4.accel_f, t0, tf, t);
}

void get_xyz()
{
  if (N_STATES == 6)
  {
    // walking states - breaks step motion into 6 points (states)
    if (state == 1)
    {
      // positions
      x1.pos_f = x_dist;
      yy1.pos_f = steer2;
      z1.pos_f = leg_height;
      x2.pos_f = (float).5 * -x_dist;
      yy2.pos_f = (float).5 * -steer2;
      z2.pos_f = leg_height - .5 * step_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = (float).5 * x_dist / state_time;
      yy1.vel_f = (float).5 * steer2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = (float)x_dist / state_time / 4 * 3;
      yy2.vel_f = (float)steer2 / state_time / 4 * 3;
      z2.vel_f = (float)-step_height / state_time / 4 * 3;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 2)
    {
      x1.pos_f = 0;
      yy1.pos_f = 0;
      z1.pos_f = leg_height;
      x2.pos_f = 0;
      yy2.pos_f = 0;
      z2.pos_f = leg_height - step_height;
      ;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = (float)-x_dist / state_time;
      yy1.vel_f = (float)-steer2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = (float).5 * x_dist / state_time;
      yy2.vel_f = (float).5 * steer2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 3)
    {
      x1.pos_f = -x_dist;
      yy1.pos_f = -steer2;
      z1.pos_f = leg_height;
      x2.pos_f = (float).5 * x_dist;
      yy2.pos_f = (float).5 * steer2;
      z2.pos_f = leg_height - .5 * step_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = (float)-x_dist / state_time;
      yy1.vel_f = (float)-steer2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = (float)x_dist / state_time / 4 * 3;
      yy2.vel_f = (float)steer2 / state_time / 4 * 3;
      z2.vel_f = (float)step_height / state_time / 4 * 3;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 4)
    {
      x1.pos_f = (float).5 * -x_dist;
      yy1.pos_f = (float).5 * -steer2;
      z1.pos_f = leg_height - .5 * step_height;
      x2.pos_f = x_dist;
      yy2.pos_f = steer2;
      z2.pos_f = leg_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = (float)x_dist / state_time / 4 * 3;
      yy1.vel_f = (float)steer2 / state_time / 4 * 3;
      z1.vel_f = -step_height / state_time / 4 * 3;
      x2.vel_f = (float).5 * x_dist / state_time;
      yy2.vel_f = (float).5 * steer2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 5)
    {
      x1.pos_f = 0;
      yy1.pos_f = 0;
      z1.pos_f = leg_height - step_height;
      ;
      x2.pos_f = 0;
      yy2.pos_f = 0;
      z2.pos_f = leg_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = (float).5 * x_dist / state_time;
      yy1.vel_f = (float).5 * steer2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = (float)-x_dist / state_time;
      yy2.vel_f = (float)-steer2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 6)
    {
      x1.pos_f = (float).5 * x_dist;
      yy1.pos_f = (float).5 * steer2;
      z1.pos_f = leg_height - .5 * step_height;
      x2.pos_f = -x_dist;
      yy2.pos_f = -steer2;
      z2.pos_f = leg_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = (float)x_dist / state_time / 4 * 3;
      yy1.vel_f = (float)steer2 / state_time / 4 * 3;
      z1.vel_f = (float)step_height / state_time / 4 * 3;
      x2.vel_f = (float)-x_dist / state_time;
      yy2.vel_f = (float)-steer2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
  }
  /*-----------------------------------------------------------------
  8 mode state definitions
  -----------------------------------------------------------------*/
  if (N_STATES == 8)
  {
    if (state == 1)
    {
      // positions
      x1.pos_f = x_dist;
      yy1.pos_f = steer2;
      z1.pos_f = leg_height;
      x2.pos_f = -x_dist;
      yy2.pos_f = -steer2;
      z2.pos_f = leg_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = -x_dist / 2 / state_time;
      yy1.vel_f = -steer2 / 2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = -x_dist / 2 / state_time;
      yy2.vel_f = -steer2 / 2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 2)
    {
      // positions
      x1.pos_f = x_dist / 2;
      yy1.pos_f = steer2 / 2;
      z1.pos_f = leg_height;
      x2.pos_f = -x_dist / 2;
      yy2.pos_f = -steer2 / 2;
      z2.pos_f = leg_height - step_height / 2;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = -x_dist / 2 / state_time;
      yy1.vel_f = -steer2 / 2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = x_dist / 2 / state_time;
      yy2.vel_f = steer2 / 2 / state_time;
      z2.vel_f = -step_height * 2 / 3 / state_time;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 3)
    {
      // positions
      x1.pos_f = 0;
      yy1.pos_f = 0;
      z1.pos_f = leg_height;
      x2.pos_f = 0;
      yy2.pos_f = 0;
      z2.pos_f = leg_height - step_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = -x_dist / 2 / state_time;
      yy1.vel_f = -steer2 / 2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = x_dist / 2 / state_time;
      yy2.vel_f = steer2 / 2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 4)
    {
      // positions
      x1.pos_f = -x_dist / 2;
      yy1.pos_f = -steer2 / 2;
      z1.pos_f = leg_height;
      x2.pos_f = x_dist * 4 / 3;
      yy2.pos_f = steer2 * 4 / 3;
      z2.pos_f = leg_height - step_height / 2;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = -x_dist / 2 / state_time;
      yy1.vel_f = -steer2 / 2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = x_dist / 4 / state_time;
      yy2.vel_f = steer2 / 2 / state_time;
      z2.vel_f = step_height * 2 / 3 / state_time;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 5)
    {
      // positions
      x1.pos_f = -x_dist;
      yy1.pos_f = -steer2;
      z1.pos_f = leg_height;
      x2.pos_f = x_dist;
      yy2.pos_f = steer2;
      z2.pos_f = leg_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = -x_dist / 2 / state_time;
      yy1.vel_f = -steer2 / 2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = -x_dist / 2 / state_time;
      yy2.vel_f = -steer2 / 2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 6)
    {
      // positions
      x1.pos_f = -x_dist / 2;
      yy1.pos_f = -steer2 / 2;
      z1.pos_f = leg_height - step_height / 2;
      x2.pos_f = x_dist / 2;
      yy2.pos_f = steer2 / 2;
      z2.pos_f = leg_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = x_dist / 2 / state_time;
      yy1.vel_f = steer2 / 2 / state_time;
      z1.vel_f = -step_height * 2 / 3 / state_time;
      x2.vel_f = -x_dist / 2 / state_time;
      yy2.vel_f = -steer2 / 2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 7)
    {
      // positions
      x1.pos_f = 0;
      yy1.pos_f = 0;
      z1.pos_f = leg_height - step_height;
      x2.pos_f = 0;
      yy2.pos_f = 0;
      z2.pos_f = leg_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = x_dist / 2 / state_time;
      yy1.vel_f = steer2 / 2 / state_time;
      z1.vel_f = 0;
      x2.vel_f = -x_dist / 2 / state_time;
      yy2.vel_f = -steer2 / 2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
    if (state == 8)
    {
      // positions
      x1.pos_f = x_dist * 4 / 3;
      yy1.pos_f = steer2 * 4 / 3;
      z1.pos_f = leg_height - step_height / 2;
      x2.pos_f = x_dist / 2;
      yy2.pos_f = steer2 / 2;
      z2.pos_f = leg_height;
      x3.pos_f = -x1.pos_f;
      yy3.pos_f = yy1.pos_f;
      z3.pos_f = z1.pos_f;
      x4.pos_f = -x2.pos_f;
      yy4.pos_f = yy2.pos_f;
      z4.pos_f = z2.pos_f;
      offset();

      // velocities
      x1.vel_f = x_dist / 4 / state_time;
      yy1.vel_f = steer2 / 2 / state_time;
      z1.vel_f = step_height * 2 / 3 / state_time;
      x2.vel_f = -x_dist / 2 / state_time;
      yy2.vel_f = -steer2 / 2 / state_time;
      z2.vel_f = 0;
      x3.vel_f = x1.vel_f;
      yy3.vel_f = yy1.vel_f;
      z3.vel_f = z1.vel_f;
      x4.vel_f = x2.vel_f;
      yy4.vel_f = yy2.vel_f;
      z4.vel_f = z2.vel_f;

      // accelerations
      x1.accel_f = 0;
      yy1.accel_f = 0;
      z1.accel_f = 0;
      x2.accel_f = 0;
      yy2.accel_f = 0;
      z2.accel_f = 0;
      x3.accel_f = x1.accel_f;
      yy3.accel_f = yy1.accel_f;
      z3.accel_f = z1.accel_f;
      x4.accel_f = x2.accel_f;
      yy4.accel_f = yy2.accel_f;
      z4.accel_f = z2.accel_f;
    }
  }
}

void offset()
{
  // offsets
  x1.pos_f += 0;
  yy1.pos_f += 20;
  z1.pos_f += 0;
  x2.pos_f += 0;
  yy2.pos_f += -20;
  z2.pos_f += 5;
  x3.pos_f += 0;
  yy3.pos_f += 20;
  z3.pos_f += 0;
  x4.pos_f += 5;
  yy4.pos_f += -20;
  z4.pos_f += 0;
}