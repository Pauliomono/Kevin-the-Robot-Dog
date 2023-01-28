#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
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
  pwm.setOscillatorFrequency(26000000);
  pwm.setPWMFreq(50);
}
int pwm_time = 1300;
bool direct = 1;

void loop() {
  // put your main code here, to run repeatedly:
  
  
  
  
  if (direct){
    pwm_time += 1;
    if (pwm_time > 1700){
    Serial.println("flip2");
    direct = false;
  }
  
  }

  if (!direct){
    pwm_time -= 1;
    if (pwm_time < 1300){
    direct = 1;
  }
  }
  Serial.println(pwm_time);
  pwm.writeMicroseconds(10, pwm_time);
    delay(100);
  

}
