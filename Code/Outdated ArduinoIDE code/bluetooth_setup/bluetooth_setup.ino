
// Basic Bluetooth sketch HC-06_01
// Connect the Hc-06 module and communicate using the serial monitor
//
// The HC-06 defaults to AT mode when first powered on.
// The default baud rate is 9600
// The Hc-06 requires all AT commands to be in uppercase. NL+CR should not be added to the command string
//
 
 
#include <SoftwareSerial.h>
SoftwareSerial SerialBT(13,12); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 2. 
// Connect the HC-06 RX to the Arduino TX on pin 3 through a voltage divider.
// 
 
 
void setup() 
{
    Serial.begin(9600);
    delay(1000);
    Serial.println("Enter AT commands:");
 
    // HC-06 default serial speed is 9600
    Serial4.begin(9600);  
}
 
void loop()
{
 
    // Keep reading from HC-06 and send to Arduino Serial Monitor
    if (Serial4.available() > 0)
    {  
        Serial.write(Serial4.read());
    }
 
    // Keep reading from Arduino Serial Monitor and send to HC-06
    if (Serial.available() > 0)
    {
        Serial4.write(Serial.read());
    }
   

 
}
