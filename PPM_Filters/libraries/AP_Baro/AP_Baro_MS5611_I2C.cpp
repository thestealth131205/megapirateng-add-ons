/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	APM_MS5611.cpp - Arduino Library for MS5611-01BA01 absolute pressure sensor
	Code by Jose Julio, Pat Hickey and Jordi Muñoz. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sensor is conected to standard SPI port
	Chip Select pin: Analog2 (provisional until Jordi defines the pin)!!

	Variables:
		Temp : Calculated temperature (in Celsius degrees * 100)
		Press : Calculated pressure   (in mbar units * 100)


	Methods:
		init() : Initialization and sensor reset
		read() : Read sensor data and _calculate Temperature, Pressure
		         This function is optimized so the main host don´t need to wait
				 You can call this function in your main loop
				 Maximun data output frequency 100Hz
				 It returns a 1 if there are new data.
		get_pressure() : return pressure in mbar*100 units
		get_temperature() : return temperature in celsius degrees*100 units

	Internal functions:
		_calculate() : Calculate Temperature and Pressure (temperature compensated) in real units


*/

#include <FastSerial.h> 

#include <I2C.h>
#include "AP_Baro_MS5611_I2C.h"

/* on APM v.24 MS5661_CS is PG1 (Arduino pin 40) */
#define MS5611_ADDRESS 0x77

#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximum resolution (oversampling)

uint32_t volatile AP_Baro_MS5611_I2C::_s_D1;
uint32_t volatile AP_Baro_MS5611_I2C::_s_D2;
uint8_t  volatile AP_Baro_MS5611_I2C::_d1_count;
uint8_t  volatile AP_Baro_MS5611_I2C::_d2_count;
uint8_t  AP_Baro_MS5611_I2C::_state;
uint32_t     AP_Baro_MS5611_I2C::_timer;
bool     AP_Baro_MS5611_I2C::_sync_access;
bool     volatile AP_Baro_MS5611_I2C::_updated;

// Public Methods //////////////////////////////////////////////////////////////
// SPI should be initialized externally
bool AP_Baro_MS5611_I2C::init( AP_PeriodicProcess *scheduler )
{
	healthy = false;
	
	delay(10);
	init_hardware();
	scheduler->register_process( AP_Baro_MS5611_I2C::_update );

	return healthy;
}

void AP_Baro_MS5611_I2C::init_hardware()
{
	byte buff[22];

	if (I2c.write(MS5611_ADDRESS, CMD_MS5611_RESET, 0) != 0) {
		delay(10);
		if (I2c.write(MS5611_ADDRESS, CMD_MS5611_RESET, 0) != 0) {
			Serial.println("fail to init MS5611, CMD_MS5611_RESET");
			healthy = false;
			return;
		}
	}
  delay(10);

	C1 = _i2c_readPROM(0xA2);
	C2 = _i2c_readPROM(0xA2+2);
	C3 = _i2c_readPROM(0xA2+4);
	C4 = _i2c_readPROM(0xA2+6);
	C5 = _i2c_readPROM(0xA2+8);
	C6 = _i2c_readPROM(0xA2+10);

/*	
	Serial.println("baro cals:");
	Serial.print("C1:");
	Serial.print(C1);
	Serial.print(",C2:");
	Serial.print(C2);
	Serial.print(",C3:");
	Serial.print(C3);
	Serial.print(",C4:");
	Serial.print(C4);
	Serial.print(",C5:");
	Serial.print(C5);
	Serial.print(",C6:");
	Serial.println(C6);
*/	
	//Send a command to read Temp first
	if (I2c.write(MS5611_ADDRESS, CMD_CONVERT_D2_OSR4096) != 0) {
		Serial.println("fail to init MS5611, CMD_CONVERT_D2_OSR4096");
		healthy = false;
		return;
	}
	_timer = micros();
	_s_D1 = 0;
	_s_D2 = 0;
	_d1_count = 0;
	_d2_count = 0;
	
	_state = 0;
	Temp=0;
	Press=0;
	healthy = true;
}

uint16_t AP_Baro_MS5611_I2C::_i2c_readPROM(uint8_t addr)
{
	byte buff[2];

	if (I2c.read(MS5611_ADDRESS, addr, 2, buff) != 0) {
		healthy = false;
		return 0;
	}
	return ( (uint16_t)buff[0] << 8 | buff[1] );
}

uint32_t AP_Baro_MS5611_I2C::_i2c_read_adc()
{
	byte buff[3];

	if (I2c.read(MS5611_ADDRESS, 0, 3, buff) != 0) {
		healthy = false;
		return false;
	}
	return ( ((uint32_t)buff[0] * 65536) | ((uint32_t)buff[1] * 256) | buff[2] );
}


// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and Pressure (states!=1)
// temperature does not change so quickly...
bool AP_Baro_MS5611_I2C::_update(uint32_t tnow)
{
    if (_sync_access || (tnow - _timer < 8500)) {
	    return false; // wait for more than 8.2ms
    }

    _timer = tnow;

    if (_state == 0) {
	    _s_D2 += _i2c_read_adc();  				 // On state 1 we read temp
      _d2_count++;
      if (_d2_count == 32) {
          // we have summed 32 values. This only happens
          // when we stop reading the barometer for a long time
          // (more than 1.2 seconds)
          _s_D2 >>= 1;
          _d2_count = 16;
      }
	    _state++;
			if (I2c.write(MS5611_ADDRESS, CMD_CONVERT_D1_OSR4096) != 0) {
				healthy = false;
				return true;
			}
    } else {
	    _s_D1 += _i2c_read_adc();
      _d1_count++;
      if (_d1_count == 128) {
          // we have summed 128 values. This only happens
          // when we stop reading the barometer for a long time
          // (more than 1.2 seconds)
          _s_D1 >>= 1;
          _d1_count = 64;
      }
	    _updated = true;					                // New pressure reading
	    _state++;
      if (_state == 5) {
				if (I2c.write(MS5611_ADDRESS, CMD_CONVERT_D2_OSR4096) != 0) {
					healthy = false;
					return true ;
				}
				_state = 0;
      } else {
			if (I2c.write(MS5611_ADDRESS, CMD_CONVERT_D1_OSR4096) != 0) {
				healthy = false;
				return true;
			}
      }
    }
	return true;
}

uint8_t AP_Baro_MS5611_I2C::read()
{
    _sync_access = true;
    bool updated = _updated;
    _updated = 0;
    if (updated > 0) {
        uint32_t sD1, sD2;
        uint8_t d1count, d2count;
        // we need to disable interrupts to access
        // _s_D1 and _s_D2 as they are not atomic
        uint8_t oldSREG = SREG;
        cli();
        sD1 = _s_D1; _s_D1 = 0;
        sD2 = _s_D2; _s_D2 = 0;
        d1count = _d1_count; _d1_count = 0;
        d2count = _d2_count; _d2_count = 0;
        _updated = false;
        SREG = oldSREG;
        if (d1count != 0) {
            D1 = ((float)sD1) / d1count;
        }
        if (d2count != 0) {
            D2 = ((float)sD2) / d2count;
        }
        _pressure_samples = d1count;
        _raw_press = D1;
        _raw_temp = D2;
    }
    if (updated) {
        _last_update = millis();
    }
    _sync_access = false;
    _calculate();
    return updated ? 1 : 0;
}

// Calculate Temperature and compensated Pressure in real units (Celsius degrees*100, mbar*100).
void AP_Baro_MS5611_I2C::_calculate()
{
	float dT;
	float TEMP;
	float OFF;
	float SENS;
	float P;

	// Formulas from manufacturer datasheet
    // sub -20c temperature compensation is not included

    // we do the calculations using floating point
    // as this is much faster on an AVR2560, and also allows
    // us to take advantage of the averaging of D1 and D1 over
    // multiple samples, giving us more precision
	dT = D2-(((uint32_t)C5)<<8);
	TEMP = (dT * C6)/8388608;
	OFF = C2 * 65536.0 + (C4 * dT) / 128;
	SENS = C1 * 32768.0 + (C3 * dT) / 256;

	if (TEMP < 0) {
        // second order temperature compensation when under 20 degrees C
		float T2 = (dT*dT) / 0x80000000;
		float Aux = TEMP*TEMP;
		float OFF2 = 2.5*Aux;
		float SENS2 = 1.25*Aux;
		TEMP = TEMP - T2;
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
	}

	P = (D1*SENS/2097152 - OFF)/32768;
	Temp = TEMP + 2000;
	Press = P;
}

float AP_Baro_MS5611_I2C::get_pressure()
{
    return Press;
}

float AP_Baro_MS5611_I2C::get_temperature()
{
	// callers want the temperature in 0.1C units
	return Temp/10;
}

int32_t AP_Baro_MS5611_I2C::get_raw_pressure() {
	return _raw_press;
}

int32_t AP_Baro_MS5611_I2C::get_raw_temp() {
	return _raw_temp;
}


