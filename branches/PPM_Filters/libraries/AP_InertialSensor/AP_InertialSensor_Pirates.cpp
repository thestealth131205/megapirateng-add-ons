#include <FastSerial.h>

#include "AP_InertialSensor_Pirates.h"

#include <I2C.h>

// #define BMA_020 // do you have it?

// *********************
// I2C general functions
// *********************
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1); 
#define ITG3200_ADDRESS  0x68 // 0xD0

#define PIRATES_ALLINONE 0
#define PIRATES_FFIMU 1
#define PIRATES_FREEIMU 2
#define PIRATES_BLACKVORTEX 3 

// accelerometer scaling
#define ACCEL_SCALE_1G    (GRAVITY / 2730.0)

#define GYRO_SMPLRT_50HZ 19 // 1KHz/(divider+1)
#define GYRO_SMPLRT_100HZ 9 // 1KHz/(divider+1)
#define GYRO_SMPLRT_200HZ 4 // 1KHz/(divider+1)

#define GYRO_DLPF_CFG_5HZ 6
#define GYRO_DLPF_CFG_10HZ 5
#define GYRO_DLPF_CFG_20HZ 4
#define GYRO_DLPF_CFG_42HZ 3
#define GYRO_DLPF_CFG_98HZ 2

// ITG-3200 14.375 LSB/degree/s
const float AP_InertialSensor_Pirates::_gyro_scale = 0.0012141421; // ToRad(1/14.375)
uint8_t AP_InertialSensor_Pirates::_board_Type = PIRATES_ALLINONE;
int AP_InertialSensor_Pirates::accel_addr = 0;
const uint8_t AP_InertialSensor_Pirates::_temp_data_index = 3;
uint8_t AP_InertialSensor_Pirates::_gyro_data_index[3];
int8_t AP_InertialSensor_Pirates::_gyro_data_sign[3];

uint8_t AP_InertialSensor_Pirates::_accel_data_index[3];
int8_t AP_InertialSensor_Pirates::_accel_data_sign[3];
uint32_t AP_InertialSensor_Pirates::_micros_per_sample = 20000;
	
static volatile uint32_t _delta_time_micros = 1;   // time period overwhich samples were collected (initialise to non-zero number but will be overwritten on 2nd read in any case)


bool AP_InertialSensor_Pirates::healthy;

AP_InertialSensor_Pirates::AP_InertialSensor_Pirates(uint8_t brd)
{
  _gyro.x = 0;
  _gyro.y = 0;
  _gyro.z = 0;
  _accel.x = 0;
  _accel.y = 0;
  _accel.z = 0;
  _initialised = 0;
  _board_Type = brd;
}

uint16_t AP_InertialSensor_Pirates::_init_sensor( AP_PeriodicProcess * scheduler, Sample_rate sample_rate)
{
	if (_initialised) return _board_Type;
		
	if (_board_Type == PIRATES_ALLINONE || _board_Type == PIRATES_FREEIMU || _board_Type == PIRATES_BLACKVORTEX) {
		_gyro_data_index[0]  =  1;
		_gyro_data_index[1]  =  2;
		_gyro_data_index[2]  =  0;
		_gyro_data_sign[0]   = 1;
		_gyro_data_sign[1]   = 1;
		_gyro_data_sign[2]   = -1;
	
		_accel_data_index[0] = 4;
		_accel_data_index[1] = 5;
		_accel_data_index[2] = 6;
		_accel_data_sign[0]  = 1;
		_accel_data_sign[1]  = 1;
		_accel_data_sign[2]  = -1;
	} else if (_board_Type == PIRATES_FFIMU) {
		_gyro_data_index[0]  =  2;
		_gyro_data_index[1]  =  1;
		_gyro_data_index[2]  =  0;
		_gyro_data_sign[0]   = 1;
		_gyro_data_sign[1]   = -1;
		_gyro_data_sign[2]   = -1;
	
		_accel_data_index[0] = 5;
		_accel_data_index[1] = 4;
		_accel_data_index[2] = 6;
		_accel_data_sign[0]  = 1;
		_accel_data_sign[1]  = -1;
		_accel_data_sign[2]  = -1;
	}

	if (_board_Type == PIRATES_ALLINONE || _board_Type == PIRATES_BLACKVORTEX) {
		accel_addr = 0x41;
	} else {
		accel_addr = 0x40;
	}

	delay(50);
	hardware_init(sample_rate);
	scheduler->register_process( &AP_InertialSensor_Pirates::read );
	_initialised = 1;
		
	return _board_Type;
}

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t _sum[7];

// how many values we've accumulated since last read
static volatile uint16_t _count;

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_Pirates::update( void )
{
	int32_t sum[7];
	uint16_t count;
	float count_scale;
	
	Vector3f gyro_offset = _gyro_offset.get();
	Vector3f accel_scale = _accel_scale.get();
	Vector3f accel_offset = _accel_offset.get();

	// wait for at least 1 sample
	while (_count == 0) ; // nop

	// disable interrupts for mininum time
	uint8_t oldSREG = SREG;
	cli();
	for (int i=0; i<7; i++) {
		sum[i] = _sum[i];
		_sum[i] = 0;
	}
	count = _count;
	_count = 0;
	// record sample time
	_delta_time_micros = count * _micros_per_sample;

	SREG = oldSREG;

	count_scale = 1.0 / count;
	_gyro.x = _gyro_scale * _gyro_data_sign[0] * sum[_gyro_data_index[0]] * count_scale;
	_gyro.y = _gyro_scale * _gyro_data_sign[1] * sum[_gyro_data_index[1]] * count_scale;
	_gyro.z = _gyro_scale * _gyro_data_sign[2] * sum[_gyro_data_index[2]] * count_scale;
	_gyro -= gyro_offset;
	
	_accel.x = accel_scale.x * _accel_data_sign[0] * sum[_accel_data_index[0]] * count_scale * ACCEL_SCALE_1G;
	_accel.y = accel_scale.y * _accel_data_sign[1] * sum[_accel_data_index[1]] * count_scale * ACCEL_SCALE_1G;
	_accel.z = accel_scale.z * _accel_data_sign[2] * sum[_accel_data_index[2]] * count_scale * ACCEL_SCALE_1G;
	_accel -= accel_offset;
	
	return true;
}

bool AP_InertialSensor_Pirates::new_data_available( void )
{
    return _count != 0;
}

// get number of samples read from the sensors
uint16_t AP_InertialSensor_Pirates::num_samples_available()
{
    return _count;
}

// get_delta_time returns the time period in seconds overwhich the sensor data was collected
uint32_t AP_InertialSensor_Pirates::get_delta_time_micros() 
{
    return _delta_time_micros;
}
/*================ HARDWARE FUNCTIONS ==================== */

static volatile uint32_t _ins_timer = 0;

bool AP_InertialSensor_Pirates::read(uint32_t tnow)
{
	if (tnow - _ins_timer < _micros_per_sample) {
		return false; // wait for more than 5ms
	}
	
	_ins_timer = tnow;
		
	static uint8_t i;
	uint8_t rawADC_ITG3200[8];
	uint8_t rawADC_BMA180[6];

	if (I2c.read(ITG3200_ADDRESS, 0X1B, 8, rawADC_ITG3200) != 0) {
		healthy = false;
		return true;
	}
  
	_sum[3] += ((rawADC_ITG3200[0]<<8) | rawADC_ITG3200[1]); // temperature
	_sum[0] += ((rawADC_ITG3200[6]<<8) | rawADC_ITG3200[7]); //g yaw
	_sum[1] += ((rawADC_ITG3200[4]<<8) | rawADC_ITG3200[5]); //g roll
	_sum[2] += ((rawADC_ITG3200[2]<<8) | rawADC_ITG3200[3]); //g pitch

	if (I2c.read(accel_addr, 0x02, 6, rawADC_BMA180) != 0) {
		healthy = false;
		return true;
	} 
	  
	_sum[4] += ((rawADC_BMA180[3]<<8) | (rawADC_BMA180[2])) >> 2; //a pitch
	_sum[5] += ((rawADC_BMA180[1]<<8) | (rawADC_BMA180[0])) >> 2; //a roll
	_sum[6] += ((rawADC_BMA180[5]<<8) | (rawADC_BMA180[4])) >> 2; //a yaw

  _count++;
  if (_count == 0) {
	  // rollover - v unlikely
	  memset((void*)_sum, 0, sizeof(_sum));
  }
 	return true;
}

void AP_InertialSensor_Pirates::hardware_init(Sample_rate sample_rate)
{
	int i;
	
	I2c.begin();
	I2c.setSpeed(true);// 400Hz
	I2c.pullup(true);

	// sample rate and filtering
	uint8_t rate, filter, default_filter;
	
	// to minimise the effects of aliasing we choose a filter
	// that is less than half of the sample rate
	switch (sample_rate) {
	case RATE_50HZ:
		rate = GYRO_SMPLRT_50HZ;
		default_filter = GYRO_DLPF_CFG_20HZ;
		_micros_per_sample = 20000;
		break;
	case RATE_100HZ:
		rate = GYRO_SMPLRT_100HZ;
		default_filter = GYRO_DLPF_CFG_20HZ;
		_micros_per_sample = 10000;
		break;
	case RATE_200HZ:
	default:
		rate = GYRO_SMPLRT_200HZ;
		default_filter = GYRO_DLPF_CFG_42HZ;
		_micros_per_sample = 5000;
		break;
	}
		
	// choose filtering frequency
	switch (_mpu6000_filter) {
	case 5:
		filter = GYRO_DLPF_CFG_5HZ;
		break;
	case 10:
		filter = GYRO_DLPF_CFG_10HZ;
		break;
	case 20:
		filter = GYRO_DLPF_CFG_20HZ;
		break;
	case 42:
		filter = GYRO_DLPF_CFG_42HZ;
		break;
	case 98:
		filter = GYRO_DLPF_CFG_98HZ;
		break;
	case 0:
	default:
	    // the user hasn't specified a specific frequency,
	    // use the default value for the given sample rate
	    filter = default_filter;
	}
		
	// GYRO
	//=== ITG3200 INIT
	delay(10);  
	if (I2c.write(ITG3200_ADDRESS, 0x3E, 0x80) != 0) {	// Power Management register, reset device
		healthy = false;
		return;
	} 	
	delay(5);
	
	if (I2c.write(ITG3200_ADDRESS, 0x15, &rate, 1) != 0) {	// Sample Rate Divider, 1000Hz/(rate+1) 
		healthy = false;
		return;
	} 	
	delay(5);
 	
	if (I2c.write(ITG3200_ADDRESS, 0x16, 0x18 + filter) != 0) {	// Internal Sample Rate 1kHz, Low pass filter: 1..6: 1=200hz, 2-100,3-50,4-20,5-10,6-5
		healthy = false;
		return;
	} 

	delay(5);
	if (I2c.write(ITG3200_ADDRESS, 0x3E, 0x03) != 0) {	// PLL with Z Gyro reference
		healthy = false;
		return;
	} 	
	delay(100);

	// ACCEL
	//===BMA180 INIT
	if (I2c.write(accel_addr, 0x0D, 1<<4) != 0) {	// ctrl_reg0, Set bit 4 to 1 to enable writing
		healthy = false;
		return;
	} 	
	if (I2c.write(accel_addr, 0x35, 3<<1) != 0) {	// range set to 3.  2730 1G raw data.  With /10 divisor on acc_ADC, more in line with other sensors and works with the GUI
		healthy = false;
		return;
	} 	
	if (I2c.write(accel_addr, 0x20, 0<<4) != 0) {	// bw_tcs reg: bits 4-7 to set bw, bw to 10Hz (low pass filter)
		healthy = false;
		return;
	} 	
	
	delay(10);  

	healthy = true;
}

float AP_InertialSensor_Pirates::_temp_to_celsius ( uint16_t regval )
{
	return (35.0 + ((float) (regval + 13200)) / 280);
}

// return the MPU6k gyro drift rate in radian/s/s
// note that this is much better than the oilpan gyros
float AP_InertialSensor_Pirates::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}
