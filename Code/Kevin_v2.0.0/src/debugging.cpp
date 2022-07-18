#include <debugging.h>

void debug_timer(){

    Serial.print(debug_point);
    Serial.print("\t");
    Serial.println(micros() -debug_time);
    
    debug_time = micros();

    debug_point ++;

    if(debug_point > n_debug_points){
        debug_point = 1;
    }
}