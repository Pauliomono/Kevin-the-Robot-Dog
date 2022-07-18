#include <servos.h>

void write_servos(){
  hip1.write(85 + leg1.hip);
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
}