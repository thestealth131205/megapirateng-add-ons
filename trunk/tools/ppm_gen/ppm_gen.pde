#include "Arduino.h"

/*
This is very simply PPM_SUM signal generator
It can be used to test PPM decoders
*/

// Output pin (CAMERA TRIGGER on Crius board)
#define OUTPIN 46
// Frame size in us
#define FRAME_SIZE 18000 
//#define FRAME_SIZE 27000 
// Low pulse width
#define LOW_WIDTH 330
// Output channels count
#define NUM_CHANNELS 4

void setup(void)
{
	pinMode(OUTPIN,OUTPUT);
	digitalWrite(OUTPIN, 1);
}

uint16_t packet[16]={1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};

void
loop(void)
{
	uint16_t frame_size = FRAME_SIZE;
	for(uint8_t i=0; i<NUM_CHANNELS; i++)
	{
		digitalWrite(OUTPIN, 0);
		delayMicroseconds(LOW_WIDTH-20);
		digitalWrite(OUTPIN, 1);
		delayMicroseconds(packet[i]-LOW_WIDTH);
		frame_size -= packet[i];
	}
	digitalWrite(OUTPIN, 0);
	delayMicroseconds(LOW_WIDTH-20);
	digitalWrite(OUTPIN, 1);
	frame_size -= LOW_WIDTH;
        if (frame_size > 16000) {
	  delay(frame_size/1000);
        } else {
  	  delayMicroseconds(frame_size);
        }
}



