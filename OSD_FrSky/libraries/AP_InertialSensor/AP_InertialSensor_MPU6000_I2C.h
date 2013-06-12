/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_INERTIAL_SENSOR_MPU6000_I2C_H__
#define __AP_INERTIAL_SENSOR_MPU6000_I2C_H__

#include <string.h>
#include <stdint.h>

#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include "../AP_Math/AP_Math.h"
#include "AP_InertialSensor.h"

class AP_InertialSensor_MPU6000_I2C : public AP_InertialSensor
{
public:

  AP_InertialSensor_MPU6000_I2C(uint8_t addr, uint8_t brd);

  uint16_t _init_sensor( AP_PeriodicProcess * scheduler, Sample_rate sample_rate );

  /* Concrete implementation of AP_InertialSensor functions: */
  bool	update();
  bool	new_data_available();
  float	get_gyro_drift_rate();
  
  // get number of samples read from the sensors
  uint16_t	num_samples_available();
  // get_delta_time returns the time period in seconds overwhich the sensor data was collected
  uint32_t	get_delta_time_micros();

private:

  static bool read(uint32_t);
  void hardware_init(Sample_rate sample_rate);

  uint32_t _last_sample_micros;
  static uint32_t _micros_per_sample;
  static uint32_t _micros_per_sample_pre;

  float _temp_to_celsius( uint16_t );

  static const float _gyro_scale;
  static uint8_t _gyro_data_index[3];
  static int8_t _gyro_data_sign[3];

  static uint8_t _accel_data_index[3];
  static int8_t _accel_data_sign[3];

  static const uint8_t _temp_data_index;

  static int16_t _data[7];

  static uint8_t _board_Type;
  static int mpu_addr;

  // ensure we can't initialise twice
  unsigned _initialised:1;
};

#endif // __AP_INERTIAL_SENSOR_MPU6000_I2C_H__
