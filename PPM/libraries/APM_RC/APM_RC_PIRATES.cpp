/*
 *	APM_RC.cpp - Radio Control Library for ArduPirates Arduino Mega with IPWM
 *	
 *	Code by Syberian, Sir Alex, SovGVD
 *	
 *	Methods:
 *		Init() : Initialization of interrupts an Timers
 *		OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..10
 *		InputCh(ch) : Read a channel input value.  ch=0..7
 *		GetState() : Returns the state of the input. 1 => New radio frame to process
 *		             Automatically resets when we call InputCh to read channels
 *		
 */
//********************************************************************************
// 2013-03-18 PAKU
// - variables structure redefined & speeded up
// - PPM Decoder rewritten
// - FS_ENABLED bug fixed
// - SYNCH frame period limit added for fine tuning Rx SYNCH period.




#include "APM_RC_PIRATES.h"

#include <avr/interrupt.h>

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

// FAILSAFE SETTINGS
// This FailSafe will detect signal loss (or receiver power failure) on Throttle pin
// In order to work properly, you must also enable Failsafe in Mission Planner
#define FS_ENABLED DISABLE

// PPM_SUM filtering
#define FILTER FILTER_DISABLED
/*
	FILTER_DISABLED
	FILTER_AVERAGE
	FILTER_JITTER
*/
#define JITTER_THRESHOLD 4
#define AVARAGE_FACTOR 1 		//changes the influance of the current meassured value to the final filter value -
								//bigger means less influance
								//min.value 1, resonable [1,2], uneven is faster [1]
#if (AVARAGE_FACTOR < 1)
# error Wrong AVARAGE_FACTOR selected. Minimum value 1
#endif


// PAKU commented out for better Eclipse view :)
//#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
//# error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
//#else

// Variable definition for Input Capture interrupt
static uint8_t use_ppm = 0; // 0-Do not use PPM, 1 - Use PPM on A8 pin, 2- Use PPM on PL1 (CRIUS v2)
static bool bv_mode;

static uint8_t *pinRcChannel;

static volatile uint32_t _last_update = 0;

// failsafe counter
static volatile uint8_t failsafeCnt = 0;
static bool failsafe_enabled = false;
static volatile bool valid_frame = false;

// ******************
// rc functions split channels
// ******************
static volatile uint16_t rcPinValue[NUM_CHANNELS]; // Default RC values
static volatile uint16_t rcPinValueRAW[NUM_CHANNELS]; // Default RC values

//************************************************************************************
// ISR routines
//************************************************************************************

typedef void (*ISRFuncPtr)(void);
static ISRFuncPtr FireISRRoutine = 0; //here the selected ISR address will be stored

ISR(PCINT2_vect) {
	if (FireISRRoutine)
		FireISRRoutine();
}

static volatile uint16_t OCRxx1[8]={1800,1800,1800,1800,1800,1800,1800,1800};
// Software PWM generator (used for Gimbal)
ISR(TIMER5_COMPB_vect)
{ // set the corresponding pin to 1
	static char OCRstate = 7;
	OCRstate++;
	OCRstate&=15;
	if (bv_mode) {
		switch (OCRstate>>1)
		{
			case 0: if(OCRstate&1)PORTC&=(1<<5)^255; else PORTC|=(1<<5);break;	//d32, cam roll
			case 1: if(OCRstate&1)PORTC&=(1<<4)^255; else PORTC|=(1<<4);break;	//d33, cam pitch
		}
	} else {
		switch (OCRstate>>1)
		{
			case 0:	if(OCRstate&1)PORTL&=(1<<5)^255; else PORTL|=(1<<5);break;	//d44, cam Roll
			case 1:	if(OCRstate&1)PORTL&=(1<<4)^255; else PORTL|=(1<<4);break;	//d45, cam Pitch
		}
	}
	if(OCRstate&1)OCR5B+=5000-OCRxx1[OCRstate>>1]; else OCR5B+=OCRxx1[OCRstate>>1];
}

//************************************************************************************

void APM_RC_PIRATES::_ppmsum_mode_isr(void)
{ 
//	digitalWrite(46,1);
	uint16_t curr_time;
	uint16_t period_time;
	uint8_t mask;
	uint8_t pin = 1;				// for V2 and V1 compatibility
	static uint16_t last_time;
	static uint8_t PCintLast;
	static uint8_t curr_ch_number;
	static bool GotFirstSynch;


	switch (use_ppm)
	{
		case SERIAL_PPM_ENABLED:

			curr_time = TCNT5;         // 0.5us resolution
			pin = PINK;               // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
			mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
			PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
			break;
		case SERIAL_PPM_ENABLED_PL1:
			curr_time = ICR5; 		  ///PAKU that's the only diff for V2 and V1 versions :)
			break;
		default : curr_time = 0;      // in any case - do nothing
			break;
	}


	// Rising edge detection
	if (pin & 1) { 

		// it should be guaranteed to wrap around - do not need to check. (unsigned values)
		period_time = (curr_time-last_time) >> 1;
		last_time = curr_time; // Save edge time
			
		// Process channel pulse
		// Good widths ??
		if ((period_time < MAX_PULSEWIDTH) && (period_time > MIN_PULSEWIDTH) && (GotFirstSynch)) {
			if (curr_ch_number < NUM_CHANNELS) {
				#if FILTER == FILTER_DISABLED
					rcPinValueRAW[curr_ch_number] = period_time;
				#elif FILTER == FILTER_AVERAGE 
					//period_time += rcPinValueRAW[curr_ch_number];
					//rcPinValueRAW[curr_ch_number] = period_time>>1;
					rcPinValueRAW[curr_ch_number]=((AVARAGE_FACTOR*rcPinValueRAW[curr_ch_number])+period_time)/(AVARAGE_FACTOR+1);
				#elif FILTER == FILTER_JITTER 
					if (abs(rcPinValueRAW[curr_ch_number]-period_time) > JITTER_THRESHOLD)
						rcPinValueRAW[curr_ch_number] = period_time;
				#endif

			}
			// Count always even if we will get more then NUM_CHANNELS >> fault detection.
			curr_ch_number++;

			if (curr_ch_number>MAX_CH_NUM) {
				valid_frame = false;						//reset validity
				GotFirstSynch = false;						//reset decoder
			}
		}


		// Process First SYNCH
		// it's SYNCH
		// That's our first SYNCH, so make stuff ready....
		else if ((period_time > MIN_PPM_SYNCHWIDTH) && (!GotFirstSynch))
		{
			GotFirstSynch = true;
			curr_ch_number=0;
			valid_frame = false;
		}

		// Process any other SYNCH
		// it's SYNCH
		else if ((period_time > MIN_PPM_SYNCHWIDTH))
		{

			// if we have got at least 4 chs
			if (curr_ch_number>3){
				valid_frame = false;						// lock reading for writing
				for (uint8_t i=0; i<NUM_CHANNELS; i++) 		// store channels
				{
					rcPinValue[i] = rcPinValueRAW[i];
				}
				valid_frame = true;							// mark data as valid
				failsafeCnt=0; 								// reset FS counter we are OK.
				//_last_update = millis();					// enable to enable APM time FS feature
			}
			else{
				failsafeCnt++; 								// that's bad
				valid_frame = false;						// reset validity
			}
			curr_ch_number=0;								// always rest on synch
		}

		// Process FAILURE - start from beginning ....
		// that's bad - we do not want to be here at any time ....
		else {
			///failsafeCnt++;
			curr_ch_number=0;
			valid_frame = false;						//reset validity
			GotFirstSynch = false;						//reset decoder
		}
	}
//	digitalWrite(46,0);
}



void APM_RC_PIRATES::_pwm_mode_isr(void)
{ //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
	uint8_t mask;
	uint8_t pin;
	uint16_t curr_time,period_time;
	static uint8_t PCintLast;
	static uint16_t edgeTime[NUM_CHANNELS];

	curr_time = TCNT5;         // from sonar
	pin = PINK;             // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
	mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
	PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

	if (mask != 0) 
		valid_frame = true;
		
	// generic split PPM  
	// mask is pins [D0-D7] that have changed // the principle is the same on the MEGA for PORTK and [A8-A15] PINs
	// chan = pin sequence of the port. chan begins at D2 and ends at D7
	if (mask & 1<<0) {
		if (!(pin & 1<<0)) {
			period_time = (curr_time-edgeTime[0])>>1; if (MIN_PULSEWIDTH<period_time && period_time<MAX_PULSEWIDTH) rcPinValue[0] = period_time;
		} else edgeTime[0] = curr_time;
	}
	if (mask & 1<<1) {
		if (!(pin & 1<<1)) {
			period_time = (curr_time-edgeTime[1])>>1; if (MIN_PULSEWIDTH<period_time && period_time<MAX_PULSEWIDTH) rcPinValue[1] = period_time;
		} else edgeTime[1] = curr_time;
	}
	if (mask & 1<<2) { 
		if (!(pin & 1<<2)) {
			period_time = (curr_time-edgeTime[2])>>1; if (MIN_PULSEWIDTH<period_time && period_time<MAX_PULSEWIDTH) rcPinValue[2] = period_time;
		} else edgeTime[2] = curr_time;
	}
	if (mask & 1<<3) {
		if (!(pin & 1<<3)) {
			period_time = (curr_time-edgeTime[3])>>1; if (MIN_PULSEWIDTH<period_time && period_time<MAX_PULSEWIDTH) rcPinValue[3] = period_time;
		} else edgeTime[3] = curr_time;
	}
	if (mask & 1<<4) {
		if (!(pin & 1<<4)) {
			period_time = (curr_time-edgeTime[4])>>1; if (MIN_PULSEWIDTH<period_time && period_time<MAX_PULSEWIDTH) rcPinValue[4] = period_time;
		} else edgeTime[4] = curr_time;
	}
	if (mask & 1<<5) {
		if (!(pin & 1<<5)) {
			period_time = (curr_time-edgeTime[5])>>1; if (MIN_PULSEWIDTH<period_time && period_time<MAX_PULSEWIDTH) rcPinValue[5] = period_time;
		} else edgeTime[5] = curr_time;
	}
	if (mask & 1<<6) {
		if (!(pin & 1<<6)) {
			period_time = (curr_time-edgeTime[6])>>1; if (MIN_PULSEWIDTH<period_time && period_time<MAX_PULSEWIDTH) rcPinValue[6] = period_time;
		} else edgeTime[6] = curr_time;
	}
	if (mask & 1<<7) {
		if (!(pin & 1<<7)) {
			period_time = (curr_time-edgeTime[7])>>1; if (MIN_PULSEWIDTH<period_time && period_time<MAX_PULSEWIDTH) rcPinValue[7] = period_time;
		} else edgeTime[7] = curr_time;
	}
	// failsafe counter must be zero if all ok  
	if (mask & 1<<pinRcChannel[2]) {    // If pulse present on THROTTLE pin, clear FailSafe counter  - added by MIS fow multiwii (copy by SovGVD to megapirateNG)
		failsafeCnt = 0;
//		_last_update = millis();
	}
}


//######################### END RC split channels

// Constructors ////////////////////////////////////////////////////////////////

APM_RC_PIRATES::APM_RC_PIRATES(int _use_ppm, int _bv_mode, uint8_t *_pin_map)
{
	use_ppm = _use_ppm; // Use serial sum (PPM)
	bv_mode = _bv_mode; // BlackVortex mode
	pinRcChannel = _pin_map; // Channel mapping
	// Fill default RC values array, set 900 for Throttle channel and 1500 for others
	for (uint8_t i=0; i<NUM_CHANNELS; i++) {
		rcPinValue[i] = 1500;
		rcPinValueRAW[i] = 1500;
	}
	rcPinValue[pinRcChannel[2]] = MIN_PULSEWIDTH;
	rcPinValueRAW[pinRcChannel[2]] = MIN_PULSEWIDTH;
}

// Public Methods //////////////////////////////////////////////////////////////

void APM_RC_PIRATES::Init( Arduino_Mega_ISR_Registry * isr_reg )
{
	failsafe_enabled = false;
	failsafeCnt = 0;
	valid_frame = false;
	//GotFirstSynch = false;  commented as it's locally static at PPM ISR only, but .... just to remember it's value is important on INIT
	
	if (bv_mode) {
		// BlackVortex Mapping
		pinMode(32,OUTPUT);	// cam roll PC5 (Digital Pin 32)
		pinMode(33,OUTPUT);	// cam pitch PC4 (Digital Pin 33)
	} else {
		pinMode(44,OUTPUT);	// cam roll PL5 (Digital Pin 44)
		pinMode(45,OUTPUT);	// cam pitch PL4 (Digital Pin 45)
	}

	//general servo
	TCCR5A = 0; //standard mode with overflow at A and OC B and C interrupts
	TCCR5B = (1<<CS11); //Prescaler set to 8, resolution of 0.5us
	OCR5B = 3000; // Init OCR registers to nil output signal

	//motors
	digitalWrite(11,HIGH);
	pinMode(11,OUTPUT);
	digitalWrite(11,HIGH);
	digitalWrite(12,HIGH);
	pinMode(12,OUTPUT);
	digitalWrite(12,HIGH);
	TCCR1A = (1<<WGM31); 
	TCCR1B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
	OCR1A = 0xFFFF; 
	OCR1B = 0xFFFF; 
	ICR1 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,

	digitalWrite(2,HIGH);
	pinMode(2,OUTPUT);
	digitalWrite(2,HIGH);
	digitalWrite(3,HIGH);
	pinMode(3,OUTPUT);
	digitalWrite(3,HIGH);
	digitalWrite(5,HIGH);
	pinMode(5,OUTPUT);
	digitalWrite(5,HIGH);
	TCCR3A = (1<<WGM31);
	TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
	OCR3A = 0xFFFF; 
	OCR3B = 0xFFFF; 
	OCR3C = 0xFFFF; 
	ICR3 = 40000; //50hz freq

	digitalWrite(6,HIGH);
	pinMode(6,OUTPUT);
	digitalWrite(6,HIGH);
	digitalWrite(7,HIGH);
	pinMode(7,OUTPUT);
	digitalWrite(7,HIGH);
	digitalWrite(8,HIGH);
	pinMode(8,OUTPUT);
	digitalWrite(8,HIGH);
	TCCR4A = (1<<WGM31);
	TCCR4B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
	OCR4A = 0xFFFF; 
	OCR4B = 0xFFFF; 
	OCR4C = 0xFFFF; 
	ICR4 = 40000; //50hz freq

	DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
	switch (use_ppm)
	{
		case SERIAL_PPM_DISABLED:
					FireISRRoutine = _pwm_mode_isr;
					PORTK = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable internal pull ups on the PINs of PORTK
					PCMSK2 = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); // enable interrupts on A8-A15 pins;
					PCICR |= (1 << PCIE2); // PCINT2 Interrupt enable
					break;
		case SERIAL_PPM_ENABLED:  
					FireISRRoutine = _ppmsum_mode_isr;
					PORTK = (1<<PCINT16); //enable internal pull up on the SERIAL SUM pin A8
					PCMSK2 |= (1 << PCINT16); // Enable int for pin A8(PCINT16)
					PCICR |= (1 << PCIE2); // PCINT2 Interrupt enable
					break;
		case SERIAL_PPM_ENABLED_PL1:
					FireISRRoutine = 0;
					pinMode(48, INPUT); // ICP5 pin (PL1) (PPM input) CRIUS v2
					isr_reg->register_signal(ISR_REGISTRY_TIMER5_CAPT, _ppmsum_mode_isr );
					TCCR5B |= (1<<ICES5); // Input capture on rising edge 
					TIMSK5 |= (1<<ICIE5); // Enable Input Capture interrupt. Timer interrupt mask  
					PCMSK2 = 0;	// Disable INT for pin A8-A15
					break;
	}
	TIMSK5 |= (1 << OCIE5B); // Enable timer5 compareB interrupt, used in Gimbal PWM generator
}


/*
ch			3		4		1		2		7		8		10		11
=======================================================================
Pin			D2	D3	D5	D6	D7	D8	D11		D12
=======================================================================

For motor mapping, see release_notes.txt
*/
void APM_RC_PIRATES::OutputCh(uint8_t ch, uint16_t pwm)
{
	pwm = constrain(pwm,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
	pwm <<= 1;   // pwm*2;
 
	switch(ch)
	{
		case 0:  OCR3A = pwm; break; //5
		case 1:  OCR4A = pwm; break; //6
		case 2:  OCR3B = pwm; break; //2
		case 3:  OCR3C = pwm; break; //3
		case 4:  OCRxx1[1] = pwm; break; //CAM PITCH
		case 5:  OCRxx1[0] = pwm; break; //CAM ROLL
		case 6:  OCR4B = pwm; break; //7
		case 7:  OCR4C = pwm; break; //8
		case 9:  OCR1A = pwm; break;// d11
		case 10: OCR1B = pwm; break;// d12
	}
}

uint16_t APM_RC_PIRATES::OutputCh_current(uint8_t ch)
{
	uint16_t pwm=0;
	switch(ch) {
		case 0:  pwm=OCR3A; break;  //ch1
		case 1:  pwm=OCR4A; break;  //ch2
		case 2:  pwm=OCR3B; break;  //ch3
		case 3:  pwm=OCR3C; break;  //ch4
		case 4:  pwm=OCRxx1[1]; break;  //ch5
		case 5:  pwm=OCRxx1[0]; break;  //ch6
		case 6:  pwm=OCR4B; break;  //ch7
		case 7:  pwm=OCR4C; break;  //ch8
		case 9:  pwm=OCR1A; break;  //ch10
		case 10: pwm=OCR1B; break;  //ch111
	}
	return pwm>>1;
}

void APM_RC_PIRATES::enable_out(uint8_t ch)
{
	switch(ch) {
		case 0: TCCR3A |= (1<<COM3A1); break; // CH_1
		case 1: TCCR4A |= (1<<COM4A1); break; // CH_2
		case 2: TCCR3A |= (1<<COM3B1); break; // CH_3
		case 3: TCCR3A |= (1<<COM3C1); break; // CH_4
			// 4,5
		case 6: TCCR4A |= (1<<COM4B1); break; // CH_7
		case 7: TCCR4A |= (1<<COM4C1); break; // CH_8
		case 9: TCCR1A |= (1<<COM1A1); break; // CH_10
		case 10: TCCR1A |= (1<<COM1B1); break; // CH_11
	}
}

void APM_RC_PIRATES::disable_out(uint8_t ch)
{
	switch(ch) {
		case 0: TCCR3A &= ~(1<<COM3A1); break; // CH_1
		case 1: TCCR4A &= ~(1<<COM4A1); break; // CH_2
		case 2: TCCR3A &= ~(1<<COM3B1); break; // CH_3
		case 3: TCCR3A &= ~(1<<COM3C1); break; // CH_4
			// 4,5
		case 6: TCCR4A &= ~(1<<COM4B1); break; // CH_7
		case 7: TCCR4A &= ~(1<<COM4C1); break; // CH_8
		case 9: TCCR1A &= ~(1<<COM1A1); break; // CH_10
		case 10: TCCR1A &= ~(1<<COM1B1); break; // CH_11
	}
}
 
uint16_t APM_RC_PIRATES::InputCh(uint8_t ch)
{
	uint16_t result;

	result = rcPinValue[pinRcChannel[ch]];

	#if FS_ENABLED == ENABLED	
		if(failsafe_enabled && (failsafeCnt >= FS_THRESHOLD)) {
			if (ch == 2) {
				result = FS_THROTTLE_VALUE;
			} else if (ch<=3) {
				result = FS_OTHER_CHANNELS_VALUE;
			}
		}
	#endif


    // paku debug
	//if (ch==7){
	//	result = 1500+failsafeCnt;
	//}

	// Limit values to a valid range
	result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
	return(result);
}

uint8_t APM_RC_PIRATES::GetState(void)
{
	bool _tmp = valid_frame;

	valid_frame = false;				//reset validity on read, just no to read more then once.

	#if FS_ENABLED == ENABLED
		if(_tmp && !failsafe_enabled)
		{
			// Ok, we got first good packet, now we can enable failsafe and start to monitor outputs
			failsafe_enabled = true;
		}

		if (failsafe_enabled) {
			if(failsafeCnt < FS_THRESHOLD) 
				failsafeCnt++;
			else 
				_tmp = true; // Override result State in order send failsafe values to APM
		}
	#endif

	return(_tmp);
}


// InstantPWM implementation
void APM_RC_PIRATES::Force_Out(void)
{
	Force_Out0_Out1();
}

// MPNG: Pirates has another channel map than AC, so we reset counters for all motors in single function

// This function forces the PWM output (reset PWM) on Out0 and Out1 (Timer3). For quadcopters use
void APM_RC_PIRATES::Force_Out0_Out1(void)
{
  if (TCNT3>5000)  // We take care that there are not a pulse in the output
    TCNT3=39990;   // This forces the PWM output to reset in 5us (10 counts of 0.5us). The counter resets at 40000
  if (TCNT4>5000)
    TCNT4=39990; 
  if (TCNT1>5000)
    TCNT1=39990; 
}
// This function forces the PWM output (reset PWM) on Out2 and Out3 (Timer4). For quadcopters use
void APM_RC_PIRATES::Force_Out2_Out3(void)
{
}
// This function forces the PWM output (reset PWM) on Out6 and Out7 (Timer1). For quadcopters use
void APM_RC_PIRATES::Force_Out6_Out7(void)
{
}

/* --------------------- OUTPUT SPEED CONTROL --------------------- */

void APM_RC_PIRATES::SetFastOutputChannels(uint32_t chmask, uint16_t speed_hz)
{
	uint16_t icr = _map_speed(speed_hz);

	if ((chmask & ( _BV(CH_10) | _BV(CH_11))) != 0) {
		ICR1 = icr;
	}

	if ((chmask & ( _BV(CH_1) | _BV(CH_3) | _BV(CH_4))) != 0) {
		ICR3 = icr;
	}

	if ((chmask & ( _BV(CH_2) | _BV(CH_7) | _BV(CH_8))) != 0) {
		ICR4 = icr;
	}
} 

// allow HIL override of RC values
// A value of -1 means no change
// A value of 0 means no override, use the real RC values
bool APM_RC_PIRATES::setHIL(int16_t v[NUM_CHANNELS])
{
/*
	uint8_t sum = 0;
	for (uint8_t i=0; i<NUM_CHANNELS; i++) {
		if (v[i] != -1) {
			_HIL_override[i] = v[i];
		}
		if (_HIL_override[i] != 0) {
			sum++;
		}
	}
	radio_status = 1;
	if (sum == 0) {
		return 0;
	} else {
		return 1;
	}
*/
	return 1;
}

void APM_RC_PIRATES::clearOverride(void)
{
	for (uint8_t i=0; i<NUM_CHANNELS; i++) {
		_HIL_override[i] = 0;
	}
}


uint32_t APM_RC_PIRATES::get_last_update() {
    return _last_update;
}; 

/// PAKU endif
///#endif // defined(ATMega1280)
