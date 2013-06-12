/*
Programmable LED sequencer for MegaPirate B8+ by Syberian
#########
- 4 programs selectable by RC channel
- each program may consist of up to 64 states
- up to 8 split LED channels
- automatic sequence restart
#########
Usage of sequencer:
Flip the Tx control switch on. The first program will be selected
Flip the switch off to dim the LEDs
Flip the switch off and on again in less than 1 second to change the program
#########

Programming instructions:
entry format (2 numbers) A,B,
A - timespan in 1/50 of one second
B - LED channels ON mask  LED_1,CH,2...LED_8 delimited by | sign
if LEDs should be off, put LED_OFF instead
Last record should end by 0,LED_OFF,   - this is mandatory!

*/

//#include <APM_RC.h>

#define LED_1 1
#define LED_2 2
#define LED_3 4
#define LED_4 8
#define LED_5 16
#define LED_6 32
#define LED_7 64
#define LED_8 128
#define LED_OFF 0


// Define Pin numbers for LEDs (unused LED channels should be commented
#define SQ_LED1 55 // A1
#define SQ_LED2 56 // A2
#define SQ_LED3 57 // A3
#define SQ_LED4 58 // A4
//#define SQ_LED1 51
//#define SQ_LED1 51
//#define SQ_LED1 51
//#define SQ_LED1 51

// Define RX channel to control 
#define SQ_LED_CH 8 // ch7 is used for SIMPLE MODE selection


// LED program # 1
const byte led_seq1[]= // 2 short 1, 2 short 2, led 4 constantly on
		{	2,	LED_1|LED_4,
			2,		 LED_4,
			2,	LED_1|LED_4,
			20,		 LED_4,
			2,	LED_2|LED_4,
			2,		 LED_4,
			2,	LED_2|LED_4,
			20,		 LED_4,
			0,	LED_OFF,
		};

// LED program # 2
const byte led_seq2[]= // 2 short 1, 2 short 2
		{	2,	LED_1,
			2,	LED_OFF,
			2,	LED_1,
			10,	LED_OFF,
			2,	LED_2,
			2,	LED_OFF,
			2,	LED_2,
			10,	LED_OFF,
			0,	LED_OFF,
		};
		

// LED program # 3
const byte led_seq3[]= // 1,2,4 blinks fast
		{	1,	LED_1|LED_2|LED_4,
			1,	LED_OFF,
			0,	LED_OFF,
		};

// LED program # 4
const byte led_seq4[]= // all the LEDs blinks at 1Hz
		{	10,	LED_1|LED_2|LED_3|LED_4|LED_5|LED_6|LED_7|LED_8,
			10,	LED_OFF,
			0,	LED_OFF,
		};


long led_hb=0;
static byte sq_led_span=0, sq_led_ctr=0, sq_led_sqnum=0,sq_led_prognum=0, sq_led_state=0;
byte sq_led_seq[128];


void sq_led_init(void)
{
#ifdef SQ_LED1
pinMode(SQ_LED1,OUTPUT);
#endif
#ifdef SQ_LED2
pinMode(SQ_LED2,OUTPUT);
#endif
#ifdef SQ_LED3
pinMode(SQ_LED3,OUTPUT);
#endif
#ifdef SQ_LED4
pinMode(SQ_LED4,OUTPUT);
#endif
#ifdef SQ_LED5
pinMode(SQ_LED5,OUTPUT);
#endif
#ifdef SQ_LED6
pinMode(SQ_LED6,OUTPUT);
#endif
#ifdef SQ_LED7
pinMode(SQ_LED7,OUTPUT);
#endif
#ifdef SQ_LED8
pinMode(SQ_LED8,OUTPUT);
#endif
// load program0 into the buffer
for (sq_led_ctr=0;sq_led_ctr<sizeof(led_seq1);sq_led_ctr++) sq_led_seq[sq_led_ctr]=led_seq1[sq_led_ctr];
led_hb=0;
#if (SQ_LED_CH<6)
#error "LED seqiencer control should be assigned to ch6 and above!"
#endif
}


void sq_led_heartbeat(void) // 20Hz loop led sequencer
{ static long tmp=0,rc_ts=0;
static int tmp2=0,tmp01=0,tmp02=0;
static byte led_ena=1;

tmp=millis();


// sequencer
// mode selection and load the program

tmp2=APM_RC.InputCh(SQ_LED_CH-1); // read the RC value
if (tmp2>1500) tmp01=1; else tmp01=0;
if (tmp01!=tmp02) //only toggle states
	{
	if (tmp01==0) {rc_ts=tmp;led_ena=0;}
	else { // enable indication or load a new program when  off pulse shorter than 1 sec
                  led_ena=1;
		if ((tmp-rc_ts)>1000) {sq_led_sqnum=0;}// just ON
		else { //load a new sequence (4 programs)
			sq_led_prognum++;sq_led_prognum&=3;
				sq_led_sqnum=0;
			switch(sq_led_prognum)
				{
				case 0:for (tmp2=0;tmp2<sizeof(led_seq1);tmp2++) sq_led_seq[tmp2]=led_seq1[tmp2]; break;
				case 1:for (tmp2=0;tmp2<sizeof(led_seq2);tmp2++) sq_led_seq[tmp2]=led_seq2[tmp2]; break;
				case 2:for (tmp2=0;tmp2<sizeof(led_seq3);tmp2++) sq_led_seq[tmp2]=led_seq3[tmp2]; break;
				case 3:for (tmp2=0;tmp2<sizeof(led_seq4);tmp2++) sq_led_seq[tmp2]=led_seq4[tmp2]; break;
				
				
				}
			}
	
	
	
	
		}
	}
tmp02=tmp01;


// end mode selection


sq_led_ctr++;
if (sq_led_ctr>sq_led_span) // load next seq step
		{sq_led_ctr=0;
		sq_led_sqnum+=2;
		if (sq_led_seq[sq_led_sqnum]==0) sq_led_sqnum=0; // reset at the end of program
		sq_led_span=sq_led_seq[sq_led_sqnum];
		if (led_ena) sq_led_state=sq_led_seq[sq_led_sqnum+1]; else sq_led_state=0;
		}
// update LEDs
#ifdef SQ_LED1
if (sq_led_state & 1) digitalWrite(SQ_LED1,HIGH); else digitalWrite(SQ_LED1,LOW);
#endif
#ifdef SQ_LED2
if (sq_led_state & 2) digitalWrite(SQ_LED2,HIGH); else digitalWrite(SQ_LED2,LOW);
#endif
#ifdef SQ_LED3
if (sq_led_state & 4) digitalWrite(SQ_LED3,HIGH); else digitalWrite(SQ_LED3,LOW);
#endif
#ifdef SQ_LED4
if (sq_led_state & 8) digitalWrite(SQ_LED4,HIGH); else digitalWrite(SQ_LED4,LOW);
#endif
#ifdef SQ_LED5
if (sq_led_state & 16) digitalWrite(SQ_LED5,HIGH); else digitalWrite(SQ_LED5,LOW);
#endif
#ifdef SQ_LED6
if (sq_led_state & 32) digitalWrite(SQ_LED6,HIGH); else digitalWrite(SQ_LED6,LOW);
#endif
#ifdef SQ_LED7
if (sq_led_state & 64) digitalWrite(SQ_LED7,HIGH); else digitalWrite(SQ_LED7,LOW);
#endif
#ifdef SQ_LED8
if (sq_led_state & 128) digitalWrite(SQ_LED8,HIGH); else digitalWrite(SQ_LED8,LOW);
#endif
}

