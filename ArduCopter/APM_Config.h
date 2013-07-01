// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __ARDUCOPTER_APMCONFIG_H__
#define __ARDUCOPTER_APMCONFIG_H__ 

// Select your sensor board
#define PIRATES_SENSOR_BOARD PIRATES_CRIUS_AIO_PRO_V1
/*
	PIRATES_ALLINONE
	PIRATES_FFIMU
	PIRATES_FREEIMU
	PIRATES_BLACKVORTEX
	PIRATES_FREEIMU_4 					// New FreeIMU 0.4.1 with MPU6050, MS5611 and 5883L
	PIRATES_DROTEK_10DOF_MPU		// MPU6050, MS5611 and 5883L
	PIRATES_CRIUS_AIO_PRO_V1		// Crius AllInOne Pro v1(1.1)
	PIRATES_CRIUS_AIO_PRO_V2		// Crius AllInOne Pro v2
*/

// RC configuration

// PPM_SUM(CPPM) Signal processing

#define SERIAL_PPM SERIAL_PPM_ENABLED
/*
	SERIAL_PPM_DISABLED             // Std.Rx using more then ONE cable to connect
	SERIAL_PPM_ENABLED				// For all boards, PPM_SUM pin is A8 (no longer the PL1 is no longer supported for AIO v2)
*/

#define TX_CHANNEL_SET	TX_JR
/*
	TX_set1							//Graupner/Spektrum												PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,CAMPITCH,CAMROLL
	TX_standard					//standard  PPM layout Robbe/Hitec/Sanwa	ROLL,PITCH,THROTTLE,YAW,MODE,AUX2,CAMPITCH,CAMROLL
	TX_set2							//some Hitec/Sanwa/others									PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL
	TX_mwi							//MultiWii layout													ROLL,THROTTLE,PITCH,YAW,AUX1,AUX2,CAMPITCH,CAMROLL
  TX_JR               //JR layout                               FLAPS:MODE, GEAR:SAVE TRIMM = apm ch7
*/

// Select your baro sensor
#define CONFIG_BARO AP_BARO_MS5611_I2C
/*
	AP_BARO_BMP085_PIRATES
	AP_BARO_MS5611_I2C
*/

// Warning: COPTER_LEDS is not compatible with LED_SEQUENCER, so enable only one option
// Connect LEDs to A4 - A7
#define COPTER_LEDS ENABLED				// Native ArduCopter LEDs
//#define LED_SEQUENCER ENABLED		// Old Syberian's LED Sequencer, see leds.pde for more info

//### PAKU #########################################################################
// Below there are CPU stress optimization defines.
// Leave Sonar and Gimbal DISABLED if you are not going to use it.
//##################################################################################
// WARNING: Sonar is fully disabled by default now.
#define CONFIG_SONAR DISABLED
/*
 	 DISABLED
 	 ENABLED
 */
#define MAX_SONAR_RANGE 400

// WARNING: Gimbal is fully disabled by default now.
#define CONFIG_GIMBAL DISABLED
/*
 	 DISABLED
 	 ENABLED
 */

// WARNING: Limits code is disabled by default now.
#define AP_LIMITS DISABLED
/*
 	 DISABLED
 	 ENABLED
 */


// This OSD works on the Serial1 port
#define OSD_PROTOCOL OSD_PROTOCOL_FRSKY
/*
	OSD_PROTOCOL_NONE
	OSD_PROTOCOL_SYBERIAN
	OSD_PROTOCOL_REMZIBI  // Read more at: http://www.rcgroups.com/forums/showthread.php?t=921467
	OSD_PROTOCOL_FRSKY		// FrSky Telemetry protocol
*/

// For BlackVortex, just set PIRATES_SENSOR_BOARD as PIRATES_BLACKVORTEX, GPS will be selected automatically
#define GPS_PROTOCOL GPS_PROTOCOL_NMEA
/*
	GPS_PROTOCOL_NONE 	without GPS
	GPS_PROTOCOL_NMEA
	GPS_PROTOCOL_SIRF
	GPS_PROTOCOL_UBLOX
	GPS_PROTOCOL_MTK19
	GPS_PROTOCOL_BLACKVORTEX
	GPS_PROTOCOL_AUTO	auto select GPS, may not work
*/
	
#define SERIAL0_BAUD			 115200	// Console port
#define SERIAL2_BAUD			 115200	// GPS port
#define SERIAL3_BAUD			 57600	// Telemetry (MAVLINK) port


///paku - define LEDs on FIRST firmware upload - could be changed later in Mission Planner
// Join options by "+" sign
//if you change LED_MODE in Mission Planer you will not be able to change it back to these values.
//but only by parameters save,text edit and reload in MP.

#define LEDS_STARTUP_MODE 		LEDS_MOTORS + LEDS_BEEPER + LEDS_GPS
/*
	LEDS_MOTORS
	LEDS_GPS
	LEDS_AUX
	LEDS_BEEPER
	LEDS_OSCILLATE
	LEDS_MOTORS_ON_NAV
	LEDS_GPS_ON_NAV
*/

///PAKU 
// motors off delay for STAB mode x/50sec ... give max about 100 = 2 sec
#define STAB_THR_OFF_DELAY 25

#define FRAME_CONFIG QUAD_FRAME
/*
 *  QUAD_FRAME
 *  TRI_FRAME
 *  HEXA_FRAME
 *  Y6_FRAME
 *  OCTA_FRAME
 *  OCTA_QUAD_FRAME
 *  HELI_FRAME
*/

#define FRAME_ORIENTATION X_FRAME
/*
 *  PLUS_FRAME
 *  X_FRAME
 *  V_FRAME
*/

# define CH7_OPTION		CH7_SAVE_TRIM
/*
 *  CH7_DO_NOTHING
 *  CH7_FLIP
 *  CH7_SIMPLE_MODE
 *  CH7_RTL
 *  CH7_SAVE_TRIM
 *  CH7_SAVE_WP
 *  CH7_CAMERA_TRIGGER
 */

// Inertia based contollers
//#define INERTIAL_NAV_XY ENABLED
#define INERTIAL_NAV_Z ENABLED

// agmatthews USERHOOKS
// the choice of function names is up to the user and does not have to match these
// uncomment these hooks and ensure there is a matching function on your "UserCode.pde" file
//#define USERHOOK_FASTLOOP userhook_FastLoop();
#define USERHOOK_50HZLOOP userhook_50Hz();
//#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();
//#define USERHOOK_SLOWLOOP userhook_SlowLoop();
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();
#define USERHOOK_INIT userhook_init();

// the choice of included variables file (*.h) is up to the user and does not have to match this one
// Ensure the defined file exists and is in the arducopter directory
#define USERHOOK_VARIABLES "UserVariables.h"

#if PIRATES_SENSOR_BOARD == PIRATES_CRIUS_AIO_PRO_V2
	#define LOGGING_ENABLED		ENABLED
#else
	#define LOGGING_ENABLED		DISABLED
#endif

// ************** EXPERIMENTAL FEATURES *****************

// #define LOITER_REPOSITIONING    ENABLED                         // Experimental Do Not Use
// #define LOITER_RP               ROLL_PITCH_LOITER_PR                        // Experimental Do Not Use

#endif //__ARDUCOPTER_APMCONFIG_H__
