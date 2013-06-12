/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AP_InertialSensor_MPU6000_I2C.h"

#include <I2C.h>

//Disable compass connected to AUX line
//#define DISABLE_AUX_COMPASS 1

// MPU6000 accelerometer scaling
#define MPU6000_ACCEL_SCALE_1G    (GRAVITY / 4096.0)

#define PIRATES_FREEIMU_4 4
#define PIRATES_DROTEK_10DOF_MPU 5 
#define PIRATES_CRIUS_AIO_PRO_V1 6
#define PIRATES_CRIUS_AIO_PRO_V2 7

// MPU 6000 registers
#define MPU6000_ADDR 0x68 //
#define MPUREG_WHOAMI 0x75 //
#define MPUREG_SMPLRT_DIV 0x19 //
	#define MPUREG_SMPLRT_1000HZ                             0x00
	#define MPUREG_SMPLRT_500HZ                              0x01
	#define MPUREG_SMPLRT_250HZ                              0x03
	#define MPUREG_SMPLRT_200HZ                              0x04
	#define MPUREG_SMPLRT_100HZ                              0x09
	#define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG 0x1A //
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_FIFO_EN 0x23
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                                   0x01
#       define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                               0x04
#       define BIT_I2C_MST_INT_EN                               0x08
#       define BIT_FIFO_OFLOW_EN                                0x10
#       define BIT_ZMOT_EN                                              0x20
#       define BIT_MOT_EN                                               0x40
#       define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS 0x3A
#define MPUREG_ACCEL_XOUT_H 0x3B //
#define MPUREG_ACCEL_XOUT_L 0x3C //
#define MPUREG_ACCEL_YOUT_H 0x3D //
#define MPUREG_ACCEL_YOUT_L 0x3E //
#define MPUREG_ACCEL_ZOUT_H 0x3F //
#define MPUREG_ACCEL_ZOUT_L 0x40 //
#define MPUREG_TEMP_OUT_H 0x41//
#define MPUREG_TEMP_OUT_L 0x42//
#define MPUREG_GYRO_XOUT_H 0x43 //
#define MPUREG_GYRO_XOUT_L 0x44 //
#define MPUREG_GYRO_YOUT_H 0x45 //
#define MPUREG_GYRO_YOUT_L 0x46 //
#define MPUREG_GYRO_ZOUT_H 0x47 //
#define MPUREG_GYRO_ZOUT_L 0x48 //
#define MPUREG_USER_CTRL 0x6A //
#define MPUREG_PWR_MGMT_1 0x6B //
#define MPUREG_PWR_MGMT_2 0x6C //
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_PRODUCT_ID 0x0C	// Product ID Register


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR      0x10
#define BIT_RAW_RDY_EN        0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA   0x01

											// Product ID Description for MPU6000
											// high 4 bits 	low 4 bits
											// Product Name	Product Revision
#define MPU6000_REV_A4				0x04 	// 0000			0100
#define MPU6000ES_REV_C4 			0x14 	// 0001			0100
#define MPU6000ES_REV_C5 			0x15 	// 0001			0101
#define MPU6000ES_REV_D6 			0x16	// 0001			0110
#define MPU6000ES_REV_D7 			0x17	// 0001			0111
#define MPU6000ES_REV_D8 			0x18	// 0001			1000	
#define MPU6000_REV_C4 				0x54	// 0101			0100 
#define MPU6000_REV_C5 				0x55	// 0101			0101
#define MPU6000_REV_D6 				0x56	// 0101			0110	
#define MPU6000_REV_D7 				0x57	// 0101			0111
#define MPU6000_REV_D8 				0x58	// 0101			1000
#define MPU6000_REV_D9 				0x59	// 0101			1001

/* 
   RS-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
   gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
const float AP_InertialSensor_MPU6000_I2C::_gyro_scale = (0.0174532 / 16.4);

/* pch: I believe the accel and gyro indicies are correct
 *      but somone else should please confirm.
 */
const uint8_t AP_InertialSensor_MPU6000_I2C::_temp_data_index = 3;
uint32_t AP_InertialSensor_MPU6000_I2C::_micros_per_sample = 20000;
uint32_t AP_InertialSensor_MPU6000_I2C::_micros_per_sample_pre = 20000;

static uint8_t _product_id;

uint8_t AP_InertialSensor_MPU6000_I2C::_board_Type = PIRATES_FREEIMU_4;
int AP_InertialSensor_MPU6000_I2C::mpu_addr = 0;

uint8_t AP_InertialSensor_MPU6000_I2C::_gyro_data_index[3];
int8_t AP_InertialSensor_MPU6000_I2C::_gyro_data_sign[3];

uint8_t AP_InertialSensor_MPU6000_I2C::_accel_data_index[3];
int8_t AP_InertialSensor_MPU6000_I2C::_accel_data_sign[3];

// variables to calculate time period over which a group of samples were collected
static volatile uint32_t _delta_time_micros = 1;   // time period overwhich samples were collected (initialise to non-zero number but will be overwritten on 2nd read in any case)
static volatile uint32_t _last_sample_time_micros = 0;  // time latest sample was collected

AP_InertialSensor_MPU6000_I2C::AP_InertialSensor_MPU6000_I2C(uint8_t addr, uint8_t brd)
{
	_gyro.x = 0;
	_gyro.y = 0;
	_gyro.z = 0;
	_accel.x = 0;
	_accel.y = 0;
	_accel.z = 0;
	_initialised = 0;
	_board_Type = brd;
	mpu_addr = addr; // Look at config.h for actual value

	if (_board_Type == PIRATES_FREEIMU_4) {
		_gyro_data_index[0]  = 4;		// X
		_gyro_data_index[1]  = 5;		// Y
		_gyro_data_index[2]  = 6;		// Z
		_gyro_data_sign[0]   = -1;	// -X
		_gyro_data_sign[1]   = 1;		// Y
		_gyro_data_sign[2]   = -1;	// -Z
	
		_accel_data_index[0] = 0;
		_accel_data_index[1] = 1;
		_accel_data_index[2] = 2;
		_accel_data_sign[0]  = -1;
		_accel_data_sign[1]  = 1;
		_accel_data_sign[2]  = -1;
	} else if (_board_Type == PIRATES_DROTEK_10DOF_MPU || _board_Type == PIRATES_CRIUS_AIO_PRO_V1 || _board_Type == PIRATES_CRIUS_AIO_PRO_V2) {
		_gyro_data_index[0]  = 5;	// Y
		_gyro_data_index[1]  = 4; // X
		_gyro_data_index[2]  = 6;	// Z
		_gyro_data_sign[0]   = 1;	// Y
		_gyro_data_sign[1]   = 1;	// X
		_gyro_data_sign[2]   = -1;// -Z
	
		_accel_data_index[0] = 1;
		_accel_data_index[1] = 0;
		_accel_data_index[2] = 2;
		_accel_data_sign[0]  = 1;
		_accel_data_sign[1]  = 1;
		_accel_data_sign[2]  = -1;
	}
}

uint16_t AP_InertialSensor_MPU6000_I2C::_init_sensor( AP_PeriodicProcess * scheduler, Sample_rate sample_rate )
{
	if (_initialised) return _product_id;
	_initialised = 1;
	delay(50);
	hardware_init(sample_rate);
	scheduler->register_process( &AP_InertialSensor_MPU6000_I2C::read );
	return _product_id;
}

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t _sum[7];

// how many values we've accumulated since last read
static volatile uint16_t _count;

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_MPU6000_I2C::update( void )
{
	int32_t sum[7];
	uint16_t count;
	float count_scale;
	Vector3f gyro_offset = _gyro_offset.get();
	Vector3f accel_scale = _accel_scale.get();
	Vector3f accel_offset = _accel_offset.get();

	// wait for at least 1 sample
	while (_count == 0) /* nop */;

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

	_accel.x = accel_scale.x * _accel_data_sign[0] * sum[_accel_data_index[0]] * count_scale * MPU6000_ACCEL_SCALE_1G;
	_accel.y = accel_scale.y * _accel_data_sign[1] * sum[_accel_data_index[1]] * count_scale * MPU6000_ACCEL_SCALE_1G;
	_accel.z = accel_scale.z * _accel_data_sign[2] * sum[_accel_data_index[2]] * count_scale * MPU6000_ACCEL_SCALE_1G;
	_accel -= accel_offset;

	return true;
}

bool AP_InertialSensor_MPU6000_I2C::new_data_available( void )
{
    return _count != 0;
}

// get_delta_time returns the time period in seconds overwhich the sensor data was collected
uint32_t AP_InertialSensor_MPU6000_I2C::get_delta_time_micros() 
{
    return _delta_time_micros;
}

float AP_InertialSensor_MPU6000_I2C::_temp_to_celsius ( uint16_t regval )
{
    /* TODO */
    return 20.0;
}

// return the MPU6k gyro drift rate in radian/s/s
// note that this is much better than the oilpan gyros
float AP_InertialSensor_MPU6000_I2C::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

// get number of samples read from the sensors
uint16_t AP_InertialSensor_MPU6000_I2C::num_samples_available()
{
    return _count;
}

/*================ HARDWARE FUNCTIONS ==================== */

/*
  this is called from a timer interrupt to read data from the MPU6000
  and add it to _sum[]
 */
static volatile uint32_t _ins_timer = 0;

bool AP_InertialSensor_MPU6000_I2C::read(uint32_t tnow)
{
	if (tnow - _ins_timer < _micros_per_sample_pre) {
		return false; // wait for more than 4ms
	}

	uint8_t _status = 0;
	
	// Data ready?
	if (I2c.read(mpu_addr, MPUREG_INT_STATUS, 1, &_status) != 0) { 
		return false;
	}
	if (_status && 1) {
		_ins_timer = tnow;
	
		// now read the data
		uint8_t rawMPU[14];
		
		if (I2c.read(mpu_addr, MPUREG_ACCEL_XOUT_H, 14, rawMPU) != 0) {
			return true;
		}
		
		_sum[0] += (((int16_t)rawMPU[0])<<8) | rawMPU[1]; // Accel X
		_sum[1] += (((int16_t)rawMPU[2])<<8) | rawMPU[3]; // Accel Y
		_sum[2] += (((int16_t)rawMPU[4])<<8) | rawMPU[5]; // Accel Z
//		_sum[3] += (((int16_t)rawMPU[6])<<8) | rawMPU[7]; // Temperature
		_sum[4] += (((int16_t)rawMPU[8])<<8) | rawMPU[9]; // Gyro X
		_sum[5] += (((int16_t)rawMPU[10])<<8) | rawMPU[11]; // Gyro Y
		_sum[6] += (((int16_t)rawMPU[12])<<8) | rawMPU[13]; // Gyro Z
	
		_count++;
		if (_count == 0) {
			// rollover - v unlikely
			memset((void*)_sum, 0, sizeof(_sum));
		}
		return true;
	} else
		return false;
}

void AP_InertialSensor_MPU6000_I2C::hardware_init(Sample_rate sample_rate)
{
    // Chip reset
		if (I2c.write(mpu_addr, MPUREG_PWR_MGMT_1, BIT_H_RESET) != 0) {
			return;
		} 	
    delay(100);
    // Wake up device and select GyroZ clock (better performance)
		if (I2c.write(mpu_addr, MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ) != 0) {
			return;
		} 	
    delay(1);
    // only used for wake-up in accelerometer only low power mode
		if (I2c.write(mpu_addr, MPUREG_PWR_MGMT_2, 0) != 0) {
			return;
		} 	
    delay(1);    

		uint8_t rate, filter, default_filter;
		
		// sample rate and filtering
		// to minimise the effects of aliasing we choose a filter
		// that is less than half of the sample rate
		switch (sample_rate) {
		case RATE_50HZ:
			rate = MPUREG_SMPLRT_50HZ;
			default_filter = BITS_DLPF_CFG_20HZ;
			_micros_per_sample = 20000;
			_micros_per_sample_pre = 19000;
			break;
		case RATE_100HZ:
			rate = MPUREG_SMPLRT_100HZ;
			default_filter = BITS_DLPF_CFG_42HZ;
			_micros_per_sample = 10000;
			_micros_per_sample_pre = 9000;
			break;
		case RATE_200HZ:
		default:
			rate = MPUREG_SMPLRT_200HZ;
			default_filter = BITS_DLPF_CFG_42HZ;
			_micros_per_sample = 5000;
			_micros_per_sample_pre = 4000;
			break;
		}
    
		// Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz
		if (I2c.write(mpu_addr, MPUREG_SMPLRT_DIV, &rate, 1) != 0) {
			return;
		}

		// choose filtering frequency
		switch (_mpu6000_filter) {
		case 5:
			filter = BITS_DLPF_CFG_5HZ;
			break;
		case 10:
			filter = BITS_DLPF_CFG_10HZ;
			break;
		case 20:
			filter = BITS_DLPF_CFG_20HZ;
			break;
		case 42:
			filter = BITS_DLPF_CFG_42HZ;
			break;
		case 98:
			filter = BITS_DLPF_CFG_98HZ;
			break;
		case 0:
		default:
		    // the user hasn't specified a specific frequency,
		    // use the default value for the given sample rate
		    filter = default_filter;
		}

    delay(1);
    // FS & DLPF   FS=2000º/s, DLPF = 98Hz (low pass filter)
		if (I2c.write(mpu_addr, MPUREG_CONFIG, &filter, 1) != 0) {
			return;
		}
		
    delay(1);
		if (I2c.write(mpu_addr, MPUREG_GYRO_CONFIG, BITS_FS_2000DPS) != 0) { // Gyro scale 2000º/s
			return;
		} 	
    delay(1);

		// Get chip revision
		uint8_t _prod;
		if (I2c.read(mpu_addr, MPUREG_PRODUCT_ID, 1, &_prod) != 0) { 
			return;
		}
		_product_id = _prod;
		// Select Accel scale
		if ( (_product_id == MPU6000_REV_A4) || (_product_id == MPU6000ES_REV_C4) || (_product_id == MPU6000ES_REV_C5) ||
			(_product_id == MPU6000_REV_C4)   || (_product_id == MPU6000_REV_C5)){
			// Accel scale 8g (4096 LSB/g)
			// Rev C has different scaling than rev D
			if (I2c.write(mpu_addr, MPUREG_ACCEL_CONFIG, 1<<3) != 0) { 
				return;
			} 	
		} else {
			// Accel scale 8g (4096 LSB/g)
			if (I2c.write(mpu_addr, MPUREG_ACCEL_CONFIG, 2<<3) != 0) { 
				return;
			} 	
		}
			
    delay(1);

		// Enable I2C bypass mode, to work with Magnetometer 5883L
		// Disable I2C Master mode
#ifndef DISABLE_AUX_COMPASS
		uint8_t user_ctrl;
		if (I2c.read(mpu_addr, MPUREG_USER_CTRL, 1, &user_ctrl) != 0) { 
			return;
		}
		user_ctrl = user_ctrl & ~(1 << 5); // reset I2C_MST_EN bit
		if (I2c.write(mpu_addr, MPUREG_USER_CTRL, &user_ctrl, 1) != 0) {
			return;
		}
    delay(1);

		// Enable I2C Bypass mode
		if (I2c.read(mpu_addr, MPUREG_INT_PIN_CFG, 1, &user_ctrl) != 0) { 
			return;
		}
		user_ctrl = user_ctrl | (1 << 1); // set I2C_BYPASS_EN bit
		if (I2c.write(mpu_addr, MPUREG_INT_PIN_CFG, &user_ctrl, 1) != 0) {
			return;
		}
#endif


		// configure interrupt to fire when new data arrives
		if (I2c.write(mpu_addr, MPUREG_INT_ENABLE, BIT_RAW_RDY_EN) != 0) {
			return;
		}
		delay(1);
}


