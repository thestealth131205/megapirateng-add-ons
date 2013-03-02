#include <FastSerial.h>
/*
// You can use this sketch to communicate with devices connected to Arduino board
// In exampe, you can configure GPS or Bluetooth modules
*/

FastSerialPort0(Serial);
FastSerialPort2(Serial2);

void setup(void)
{
	// Computer <-> Arduino speed
	Serial.begin(115200);

	// Arduino <-> your device
	Serial2.begin(9600);
	
	Serial.println("Started");
}

void
loop(void)
{
    byte    c;
    if (Serial2.available()){
    	c = Serial2.read();
      Serial.write(c);
    }
    if (Serial.available()){
    	c = Serial.read();
      Serial2.write(c);
    }
}



