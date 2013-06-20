/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_HMC5843_Pirates_H
#define AP_Compass_HMC5843_Pirates_H
 
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_PeriodicProcess/AP_PeriodicProcess.h" 

#include "Compass.h"

// orientations for DIYDrones magnetometer
#define AP_COMPASS_COMPONENTS_UP_PINS_FORWARD ROTATION_NONE
#define AP_COMPASS_COMPONENTS_UP_PINS_FORWARD_RIGHT ROTATION_YAW_45
#define AP_COMPASS_COMPONENTS_UP_PINS_RIGHT ROTATION_YAW_90
#define AP_COMPASS_COMPONENTS_UP_PINS_BACK_RIGHT ROTATION_YAW_135
#define AP_COMPASS_COMPONENTS_UP_PINS_BACK ROTATION_YAW_180
#define AP_COMPASS_COMPONENTS_UP_PINS_BACK_LEFT ROTATION_YAW_225
#define AP_COMPASS_COMPONENTS_UP_PINS_LEFT ROTATION_YAW_270
#define AP_COMPASS_COMPONENTS_UP_PINS_FORWARD_LEFT ROTATION_YAW_315
#define AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD ROTATION_ROLL_180
#define AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD_RIGHT ROTATION_ROLL_180_YAW_45
#define AP_COMPASS_COMPONENTS_DOWN_PINS_RIGHT ROTATION_ROLL_180_YAW_90
#define AP_COMPASS_COMPONENTS_DOWN_PINS_BACK_RIGHT ROTATION_ROLL_180_YAW_135
#define AP_COMPASS_COMPONENTS_DOWN_PINS_BACK ROTATION_PITCH_180
#define AP_COMPASS_COMPONENTS_DOWN_PINS_BACK_LEFT ROTATION_ROLL_180_YAW_225
#define AP_COMPASS_COMPONENTS_DOWN_PINS_LEFT ROTATION_ROLL_180_YAW_270
#define AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD_LEFT ROTATION_ROLL_180_YAW_315
#define AP_COMPASS_APM2_SHIELD ROTATION_NONE

// orientations for Sparkfun magnetometer
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_FORWARD ROTATION_YAW_270
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_FORWARD_RIGHT ROTATION_YAW_315
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_RIGHT ROTATION_NONE
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_BACK_RIGHT ROTATION_YAW_45
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_BACK ROTATION_YAW_90
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_BACK_LEFT ROTATION_YAW_135
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_LEFT ROTATION_YAW_180
#define AP_COMPASS_SPARKFUN_COMPONENTS_UP_PINS_FORWARD_LEFT ROTATION_YAW_225
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_FORWARD ROTATION_ROLL_180_YAW_90
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_FORWARD_RIGHT ROTATION_ROLL_180_YAW_135
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_RIGHT ROTATION_PITCH_180
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_BACK_RIGHT ROTATION_ROLL_180_YAW_225
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_BACK ROTATION_ROLL_180_YAW_270
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_BACK_LEFT ROTATION_ROLL_180_YAW_315
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_LEFT ROTATION_ROLL_180
#define AP_COMPASS_SPARKFUN_COMPONENTS_DOWN_PINS_FORWARD_LEFT ROTATION_ROLL_180_YAW_45

class AP_Compass_HMC5843_Pirates : public Compass
{
  private:
	void init_hardware();
	static bool read_raw(void);
	static uint8_t _base_config;
	static bool re_initialise(void);
	static bool read_register(uint8_t address, uint8_t *value);
	static bool write_register(uint8_t address, byte value);
	static long _compass_timer;
	static int _raw_mag_x;          ///< magnetic field strength along the X axis
	static int _raw_mag_y;          ///< magnetic field strength along the Y axis
	static int _raw_mag_z;          ///< magnetic field strength along the Z axis
	float calibration[3];

  public:
	AP_Compass_HMC5843_Pirates() : Compass() {}
  bool init(AP_PeriodicProcess *scheduler);
	bool init() { return false; };
	bool read(void);
	void set_orientation(enum Rotation rotation);
	static bool _updated;
	static bool _update(uint32_t tnow);
	void accumulate(void);
};
#endif