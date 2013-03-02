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
#define FS_ENABLED ENABLED

// PPM_SUM filtering
#define FILTER FILTER_JITTER
/*
	FILTER_DISABLED
	FILTER_AVERAGE
	FILTER_JITTER
*/
#define JITTER_THRESHOLD 4

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
# error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#else

// Variable definition for Input Capture interrupt
volatile uint8_t use_ppm = 0; // 0-Do not use PPM, 1 - Use PPM on A8 pin, 2- Use PPM on PL1 (CRIUS v2) 
volatile bool bv_mode;
uint8_t *pinRcChannel;

volatile uint32_t _last_update = 0;

// failsafe counter
volatile uint8_t failsafeCnt = 0;
volatile bool failsafe_enabled = false;
volatile bool valid_frame = false;

// ******************
// rc functions split channels
// ******************
volatile uint16_t rcPinValue[NUM_CHANNELS]; // Default RC values
volatile uint16_t rcPinValueRAW[NUM_CHANNELS]; // Default RC values
volatile bool good_sync_received = false;
volatile uint8_t pps_num=0;
typedef void (*ISRFuncPtr)(void);
static volatile ISRFuncPtr FireISRRoutine = 0; 

ISR(PCINT2_vect) {
	if (FireISRRoutine)
		FireISRRoutine();
}

volatile uint16_t ICR5_old;

// Serial PPM support on PL1(ICP5) pin
void APM_RC_PIRATES::_timer5_capt_cb(void)
{
	uint16_t Pulse;
	uint16_t dTime;
	
	Pulse = ICR5;
	// Calculating pulse width
	if (Pulse<ICR5_old) { 
		dTime = ((0xFFFF - ICR5_old)+Pulse) >> 1; 
	}
	else {
		dTime = (Pulse-ICR5_old) >> 1;
	}
	ICR5_old = Pulse;
	
	if (dTime < MAX_PULSEWIDTH  && dTime > MIN_PULSEWIDTH) {
		// Ignore more than NUM_CHANNELS channels
		if (pps_num < NUM_CHANNELS) {
			#if FILTER == FILTER_DISABLED
				rcPinValueRAW[pps_num] = dTime;
			#elif FILTER == FILTER_AVERAGE 
				dTime += rcPinValueRAW[pps_num];
				rcPinValueRAW[pps_num] = dTime>>1;
			#elif FILTER == FILTER_JITTER 
				if (abs(rcPinValueRAW[pps_num]-dTime) > JITTER_THRESHOLD)
					rcPinValueRAW[pps_num] = dTime;
			#endif
			pps_num++;
		}
	// Sync signal ?
	} else if (dTime > MAX_PULSEWIDTH) {
		// Skip all data before first good sync received
		if (pps_num > 3) {
			if (good_sync_received) {
				for (uint8_t i=0; i<NUM_CHANNELS; i++) {
					rcPinValue[i] = rcPinValueRAW[i];
				}
				// If we read at least 4 channel - reset failsafe counter
				failsafeCnt = 0;
				valid_frame = true;
				_last_update = millis();
			}
			good_sync_received = true;
		}
		pps_num = 0; // Reset packet to Channel 0
	}
}

volatile uint16_t pps_etime=0;
volatile uint8_t PCintLast;

void APM_RC_PIRATES::_ppmsum_mode_isr(void)
{ 
//	digitalWrite(46,1);
	uint16_t cTime,dTime;
	uint8_t mask;
	uint8_t pin;

	cTime = TCNT5; // 0.5us resolution
	pin = PINK;             // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
	mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
	PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

	// Rising edge detection
	if (pin & 1) { 
		// Calculate pulse width
		if (cTime < pps_etime)
			dTime = ((0xFFFF-pps_etime)+cTime) >> 1;
		else
			dTime = (cTime-pps_etime) >> 1; 
		pps_etime = cTime; // Save edge time
			
		// Valid pulse channel?
		if (dTime < MAX_PULSEWIDTH && dTime > MIN_PULSEWIDTH) {
			// Ignore more than NUM_CHANNELS channels
			if (pps_num < NUM_CHANNELS) {
				#if FILTER == FILTER_DISABLED
					rcPinValueRAW[pps_num] = dTime;
				#elif FILTER == FILTER_AVERAGE 
					dTime += rcPinValueRAW[pps_num];
					rcPinValueRAW[pps_num] = dTime>>1;
				#elif FILTER == FILTER_JITTER 
					if (abs(rcPinValueRAW[pps_num]-dTime) > JITTER_THRESHOLD)
						rcPinValueRAW[pps_num] = dTime;
				#endif
				pps_num++;
			}
		// Sync signal ?
		} else if (dTime > MAX_PULSEWIDTH) {
			// Skip all data before first good sync received
			if (pps_num > 3) {
				if (good_sync_received) {
					for (uint8_t i=0; i<NUM_CHANNELS; i++) {
						rcPinValue[i] = rcPinValueRAW[i];
					}
					// If we read at least 4 channel - reset failsafe counter
					failsafeCnt = 0;
					valid_frame = true;
					_last_update = millis();
				}
				good_sync_received = true;
			}
			pps_num = 0; // Reset packet to Channel 0
		}
	}
//	digitalWrite(46,0);
}

volatile uint16_t edgeTime[8];

void APM_RC_PIRATES::_pwm_mode_isr(void)
{ //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
	uint8_t mask;
	uint8_t pin;
	uint16_t cTime,dTime;

	cTime = TCNT5;         // from sonar
	pin = PINK;             // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
	mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
	PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

	if (mask != 0)
		valid_frame = true;
		
	// generic split PPM  
	// mask is pins [D0-D7] that have changed // the principle is the same on the MEGA for PORTK and [A8-A15] PINs
	// chan = pin sequence of the port. chan begins at D2 and ends at D7
	if (mask & 1<<0)
		if (!(pin & 1<<0)) {
			dTime = (cTime-edgeTime[0])>>1; if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) rcPinValue[0] = dTime;
		} else edgeTime[0] = cTime;
	if (mask & 1<<1)
		if (!(pin & 1<<1)) {
			dTime = (cTime-edgeTime[1])>>1; if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) rcPinValue[1] = dTime;
		} else edgeTime[1] = cTime;
	if (mask & 1<<2) 
		if (!(pin & 1<<2)) {
			dTime = (cTime-edgeTime[2])>>1; if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) rcPinValue[2] = dTime;
		} else edgeTime[2] = cTime;
	if (mask & 1<<3)
		if (!(pin & 1<<3)) {
			dTime = (cTime-edgeTime[3])>>1; if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) rcPinValue[3] = dTime;
		} else edgeTime[3] = cTime;
	if (mask & 1<<4) 
		if (!(pin & 1<<4)) {
			dTime = (cTime-edgeTime[4])>>1; if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) rcPinValue[4] = dTime;
		} else edgeTime[4] = cTime;
	if (mask & 1<<5)
		if (!(pin & 1<<5)) {
			dTime = (cTime-edgeTime[5])>>1; if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) rcPinValue[5] = dTime;
		} else edgeTime[5] = cTime;
	if (mask & 1<<6)
		if (!(pin & 1<<6)) {
			dTime = (cTime-edgeTime[6])>>1; if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) rcPinValue[6] = dTime;
		} else edgeTime[6] = cTime;
	if (mask & 1<<7)
		if (!(pin & 1<<7)) {
			dTime = (cTime-edgeTime[7])>>1; if (MIN_PULSEWIDTH<dTime && dTime<MAX_PULSEWIDTH) rcPinValue[7] = dTime;
		} else edgeTime[7] = cTime;
	
	// failsafe counter must be zero if all ok  
	if (mask & 1<<pinRcChannel[2]) {    // If pulse present on THROTTLE pin, clear FailSafe counter  - added by MIS fow multiwii (copy by SovGVD to megapirateNG)
		failsafeCnt = 0;
		_last_update = millis();
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
		case 0:
					FireISRRoutine = _pwm_mode_isr;
					PORTK = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable internal pull ups on the PINs of PORTK
					PCMSK2 = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); // enable interrupts on A8-A15 pins;
					PCICR |= (1 << PCIE2); // PCINT2 Interrupt enable
					break;
		case 1:  
					FireISRRoutine = _ppmsum_mode_isr;
					PORTK = (1<<PCINT16); //enable internal pull up on the SERIAL SUM pin A8
					PCMSK2 |= (1 << PCINT16); // Enable int for pin A8(PCINT16)
					PCICR |= (1 << PCIE2); // PCINT2 Interrupt enable
					break;
		case 2:
					FireISRRoutine = 0;
					pinMode(48, INPUT); // ICP5 pin (PL1) (PPM input) CRIUS v2
					isr_reg->register_signal(ISR_REGISTRY_TIMER5_CAPT, _timer5_capt_cb );
					TCCR5B = (1<<CS11) | (1<<ICES5); //Prescaler set to 8, resolution of 0.5us, input capture on rising edge 
					TIMSK5 |= (1<<ICIE5); // Enable Input Capture interrupt. Timer interrupt mask  
					PCMSK2 = 0;	// Disable INT for pin A8-A15
					break;
	}
	TIMSK5 |= (1 << OCIE5B); // Enable compareB interrupt, used in Gimbal PWM generator
}

uint16_t OCRxx1[8]={1800,1800,1800,1800,1800,1800,1800,1800,};
char OCRstate = 7;

// Software PWM generator (used for Gimbal)
ISR(TIMER5_COMPB_vect)
{ // set the corresponding pin to 1
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

	result = rcPinValue[pinRcChannel[ch]]; // Let's copy the data Atomically
	while( result != rcPinValue[pinRcChannel[ch]] ) result = rcPinValue[pinRcChannel[ch]];

	#if FS_ENABLED == ENABLED	
		if(failsafe_enabled && (failsafeCnt >= FS_THRESHOLD)) {
			if (ch == 2) {
				result = FS_THROTTLE_VALUE;
			} else if (ch<=3) {
				result = FS_OTHER_CHANNELS_VALUE;
			}
		}
	#endif
  
	// Limit values to a valid range
	result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
	return(result);
}

uint8_t APM_RC_PIRATES::GetState(void)
{
	bool _tmp = valid_frame;
	
	valid_frame = false;
	
	// If we not received good frame after 2 reads, reset status and start from scratch
	if (!_tmp) {
		good_sync_received = false;
	}

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
				return(1); // Force Good State in order send failsafe values to APM
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

// get the time of the last radio update (_last_update modified by interrupt, so reading of variable must be interrupt safe)
uint32_t APM_RC_PIRATES::get_last_update() {
    
    uint32_t _tmp = _last_update;
    while( _tmp != _last_update ) _tmp = _last_update;

    return _tmp;
}; 

#endif // defined(ATMega1280)
