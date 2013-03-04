// I2C device detector for MegaPirate Flight Controller
// by Syberian
#include <Wire.h>
#include "Arduino.h"

#ifndef CPU_FREQ
  #define CPU_FREQ 16000000L
#endif

#define sbi(sfr, bit)   (_SFR_BYTE(sfr) |= _BV(bit))

#define MPUREG_USER_CTRL 0x6A
#define MPUREG_INT_PIN_CFG 0x37 
#define MPUREG_PWR_MGMT_1 0x6B // 
#define BIT_H_RESET 0x80
#define MPU_CLK_SEL_PLLGYROZ 0x03 

// read_register - read a register value
static uint8_t 
read_register(uint8_t address, uint8_t reg_addr)
{
	uint8_t ret = 0;

	Wire.beginTransmission(address);
	Wire.write(reg_addr);     //sends address to read from
	if (0 != Wire.endTransmission())
		return 0;

	Wire.requestFrom(address, uint8_t(1));    // request 1 byte from device
	if( Wire.available() ) {
		ret = Wire.read();  // receive one byte
	}
	if (0 != Wire.endTransmission())
		return 0;

	return ret;
}

// write_register - update a register value
static bool 
write_register(byte address, int reg_addr, uint8_t value)
{
	Wire.beginTransmission(address);
	Wire.write(reg_addr);
	Wire.write(value);
	if (0 != Wire.endTransmission())
		return false;
	delay(10);
	return true;
} 

void init_mpu650(uint8_t addr)
{
	write_register(addr, MPUREG_PWR_MGMT_1, BIT_H_RESET);  
	delay(100);
	write_register(addr, MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);  
	// Disable I2C Master mode
	uint8_t user_ctrl;
	user_ctrl = read_register(addr, MPUREG_USER_CTRL);
	user_ctrl = user_ctrl & ~(1 << 5); // reset I2C_MST_EN bit
	write_register(addr, MPUREG_USER_CTRL,user_ctrl);
	delay(100);
	// Enable I2C Bypass mode
	user_ctrl = read_register(addr, MPUREG_INT_PIN_CFG);
	user_ctrl = user_ctrl | (1 << 1); // set I2C_BYPASS_EN bit
	write_register(addr, MPUREG_INT_PIN_CFG,user_ctrl);
}

void setup()
{
	Wire.begin();
    TWBR = ((CPU_FREQ / 400000) - 16) / 2;
      sbi(PORTD, 0);
      sbi(PORTD, 1);

	Serial.begin(115200);
	Serial.println("I2C devices detector");
	Serial.println("=================================");
	Serial.println();
}

void loop()
{
	Serial.println("Start new scan");
	for(int i=0;i<128;i++)  
	{
	  Wire.requestFrom(i, 1);
	  while(Wire.available())
	  { 
	    byte c = Wire.read();
	    Serial.print("Detected device addr: 0x");
	    Serial.print(i,HEX);
	    switch (i)
	    { case 0x1E: Serial.println(" HMC5883/43 (compass)");break;
	      case 0x40: Serial.println(" BMA180 (accel) FFIMU or BB");break;
	      case 0x41: Serial.println(" BMA180 (accel) Allinone board");break;
	      case 0x68: Serial.println(" ITG3200 (gyro), MPU6050 (gyro+accel)"); init_mpu650(0x68); break;
	      case 0x69: Serial.println(" MPU6050 (gyro+accel)"); init_mpu650(0x69); break;
	      case 0x77: Serial.println(" BMP085 (baro)");break;
	      default: Serial.println(" unknown device!");break;
	    }  
	  }
	  switch (i)
	  { 
	    case 0x76: 
	                if (read_register(0x76, 0xA2)!=0) { 
	                  Serial.print("Detected device addr: 0x");
	                  Serial.print(i,HEX);
	                  Serial.println(" MS5611 (baro)"); 
	                }  break;
	    case 0x77: 	
	                if (read_register(0x77, 0xA2)!=0) { 
	                  Serial.print("Detected device addr: 0x");
	                  Serial.print(i,HEX);
	                  Serial.println(" MS5611 (baro)"); 
	                }  break;
	  }
	}
	Serial.println("=================================");
	Serial.println("Cycle is over");
	Serial.println("");
	delay(1000);
}
