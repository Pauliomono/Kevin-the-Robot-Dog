#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include <Smoothed.h>
//#include <PID_AutoTune_v0.h>

int pwm_time;
int pwm_old;
int direction = 1;
double input;
double shoulder_angle;
double knee_angle;
double input_deg;
double output;
double degrees = 60;
int i;
double kp = 0;
double ki = 0;
double kd = 0;
int t;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Smoothed<float> pos_pot_hip;
Smoothed<float> pos_pot_shoulder;
Smoothed<float> pos_pot_knee;

PID PID_servo(&input, &output, &degrees, kp, ki, kd, DIRECT);

//#define HOLD
//#define SWEEP

void setup()
{

  pinMode(A8, INPUT);
  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  // put your setup code here, to run once:
  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency.
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */

  pos_pot_hip.begin(SMOOTHED_EXPONENTIAL, 3);
  pos_pot_hip.add(analogRead(A8));
  pos_pot_shoulder.begin(SMOOTHED_EXPONENTIAL, 3);
  pos_pot_shoulder.add(analogRead(A7));
  pos_pot_knee.begin(SMOOTHED_EXPONENTIAL, 3);
  pos_pot_knee.add(analogRead(A6));

  PID_servo.SetMode(AUTOMATIC);
  PID_servo.SetOutputLimits(-100, 100);
  PID_servo.SetSampleTime(1);
  pwm.setOscillatorFrequency(26000000);
  pwm.setPWMFreq(50);
}

void loop()
{

  for (double j = 0.1; j <= 10; j += .1)
  {
    //kp = j;
    PID_servo.SetTunings(kp, ki, kd);

    i = 0;

    while (i <= 1)
    {
      pos_pot_hip.add(analogRead(A8));
      pos_pot_shoulder.add(analogRead(A7));
      pos_pot_knee.add(analogRead(A6));

      input = (float)-180 / 560 * (pos_pot_hip.get() - 220) + 180;
      shoulder_angle = (float)-180 / 560 * (pos_pot_shoulder.get() - 220) + 180;
      knee_angle = (float)-180 / 560 * (pos_pot_knee.get() - 220) + 180;

      // input = map(input, 200, 800, 180, 0);

      input_deg = (float)-180 / 560 * (input - 215) + 180;

      // input = (float)-180/1024*input + 180;

#ifdef SWEEP
      if (millis() % 25 == 0)
      {
        if (degrees >= 120)
        {
          direction = 0;
        }

        if (degrees <= 60)
        {
          direction = 1;
          i++;
        }

        if (direction == 1)
        {
          degrees += .5;
        }

        if (direction == 0)
        {
          degrees -= .5;
        }
        pwm_time = map(degrees, 0, 180, 544, 2400);
      }

#endif

#ifdef HOLD
      if (millis() % 4000 == 0)
      {
        degrees = 70;
        // pwm_time = (float)1000.0 + degrees * 50.0 / 9.0;
        pwm_time = map(degrees, 0, 180, 544, 2400);
        i++;
      }

      if (millis() % 4000 == 1000)
      {
        degrees = 90;
        pwm_time = map(degrees, 0, 180, 544, 2400);
      }

      if (millis() % 4000 == 2000)
      {
        degrees = 110;
        // pwm_time = (float)800.0 + degrees * 70.0 / 9.0;
        pwm_time = map(degrees, 0, 180, 544, 2400);
      }

      if (millis() % 4000 == 3000)
      {
        degrees = 90;
        pwm_time = map(degrees, 0, 180, 544, 2400);
      }
#endif
      degrees = 90;
      pwm_time = map(degrees, 0, 180, 544, 2400);
      

      PID_servo.Compute();
      pwm_old = pwm_time;
      // pwm_time += output;

      //if (millis() % 10 == 0)
      //{
        t = millis();
        Serial.print(t);
        Serial.print("\t");
        Serial.print(input);
        Serial.print("\t");
        Serial.print(PID_servo.GetKp());
        Serial.print("\t");
        Serial.print(ki);
        Serial.print("\t");
        Serial.print(kd * 100);
        Serial.print("\t");
        Serial.print(output);
        Serial.print("\t");
        Serial.print(degrees);
        Serial.print("\t");
        Serial.print(pwm_time);
        Serial.println("\t");
      //}

      pwm.writeMicroseconds(10, pwm_time + output);
      //pwm.writeMicroseconds(10, 1500);
      pwm.writeMicroseconds(11, 1500);
      pwm.writeMicroseconds(12, 1500);
    }
  }
}