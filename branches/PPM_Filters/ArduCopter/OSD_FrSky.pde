// ****************************************************************
// FrSky telemetry protocol
// You must connect FrSky RX pin via TTL invertor to Serial1 TX (TX1)
// Code ported from MultiWii project by Sir Alex
// ****************************************************************
// 27.02.2013 by paku
// Additions:
// - missing protocol tail in counter loop 
// - Fuel Level 
// - Fuel Level - demo included
// - busy lock - while in transmission - makes code few CPU ticks/time faster ....
// - period changed to 200ms (as in FrSky docs)
// - "altitude after period" frame bytes enabled 
// Editing:
// - functions calls rearranged - commented out moved to the end of file.
// - source text auto formatted
// ****************************************************************
// 01.03.2013 by paku
// Fuel Demo changed to "Pirate state" display
// Powered			- XX
// Armed			- XXXX
// GPS Fix			- XXXXXX
// Home Set			- XXXXXXXX

#if OSD_PROTOCOL == OSD_PROTOCOL_FRSKY

// user defines
//#define FAS_100  //if commment out, MultiWii vBat voltage will be send instead of FrSky FAS 100 voltage

// Serial config datas
#define TELEMETRY_FRSKY_SERIAL 1
#define TELEMETRY_FRSKY_BAUD 9600 

// Timing
#define Time_telemetry_send 200000
static uint8_t cycleCounter = 0;
#define MaxCounter 		25
static uint32_t FrSkyTime = 0;
static uint8_t LockMe = 0x00;

// Frame protocol
#define Protocol_Header   0x5E
#define Protocol_Tail     0x5E

// Data Ids  (bp = before point; af = after point)
// Official data IDs
// Data Ids  (bp = before point; af = after point)
// Official data IDs
#define ID_GPS_altidute_bp    0x01
#define ID_GPS_altidute_ap    0x09
#define ID_Temprature1        0x02
#define ID_RPM                0x03
#define ID_Fuel_level         0x04
#define ID_Temprature2        0x05
#define ID_Volt               0x06
#define ID_Altitude_bp        0x10
#define ID_Altitude_ap        0x21
#define ID_GPS_speed_bp       0x11
#define ID_GPS_speed_ap       0x19
#define ID_Longitude_bp       0x12
#define ID_Longitude_ap       0x1A
#define ID_E_W                0x22
#define ID_Latitude_bp        0x13
#define ID_Latitude_ap        0x1B
#define ID_N_S                0x23
#define ID_Course_bp          0x14
#define ID_Course_ap          0x1C
#define ID_Date_Month         0x15
#define ID_Year               0x16
#define ID_Hour_Minute        0x17
#define ID_Second             0x18
#define ID_Acc_X              0x24
#define ID_Acc_Y              0x25
#define ID_Acc_Z              0x26
#define ID_Voltage_Amp_bp     0x3A
#define ID_Voltage_Amp_ap     0x3B
#define ID_Current            0x28
// User defined data IDs
#define ID_Gyro_X             0x40
#define ID_Gyro_Y             0x41
#define ID_Gyro_Z             0x42

// Main function FrSky telemetry
void telemetry_frsky() {
	uint32_t currentTime = micros();
	if (currentTime > FrSkyTime) {
		LockMe = 0xff;
		FrSkyTime = currentTime + Time_telemetry_send;
		cycleCounter++;
		// Datas sent every Time_telemetry_send 
		//send_Altitude();
		//send_Accel();
		//sendDataTail();   

		if ((cycleCounter % 4) == 0) {
			// Datas sent every 4*Time_telemetry_send
			send_Time();
			send_Altitude();
			//send_Course();
			send_GPS_speed();
			send_Fuel_level();
			sendDataTail();

		}
		if ((cycleCounter % 8) == 0) {
			// Datas sent every 8*Time_telemetry_send           
			//send_GPS_altitude();
			//send_Voltage_ampere();
			send_GPS_position();
			send_Temperature2();  // num of Sats
			sendDataTail();
		}

		if (cycleCounter == MaxCounter) {
			// Datas sent every 25*Time_telemetry_send
			send_Temperature1();
			sendDataTail();
			cycleCounter = 0;
		}
		LockMe = 0x00;
	}
}

void write_FrSky8_internal(uint8_t Data) {
	Serial1.write(Data);
}

void write_FrSky8(uint8_t Data) {
	check_FrSky_stuffing(Data);
}

void write_FrSky16(uint16_t Data) {
	uint8_t Data_send;
	Data_send = Data;
	check_FrSky_stuffing(Data_send);
	Data_send = Data >> 8 & 0xff;
	check_FrSky_stuffing(Data_send);
}

void check_FrSky_stuffing(uint8_t Data) {
	if (Data == 0x5E) {
		write_FrSky8_internal(0x5D);
		write_FrSky8_internal(0x3E);
	} else if (Data == 0x5D) {
		write_FrSky8_internal(0x5D);
		write_FrSky8_internal(0x3D);
	} else {
		write_FrSky8_internal(Data);
	}
}

static void sendDataHead(uint8_t Data_id) {
	write_FrSky8_internal(Protocol_Header);
	write_FrSky8_internal(Data_id);
}

static void sendDataTail(void) {
	write_FrSky8_internal(Protocol_Tail);
}

//*********************************************************************************
//-----------------   Telemetrie Datas   ------------------------------------------   
//*********************************************************************************

// Voltage (Ampere Sensor) 
void send_Voltage_ampere(void) {
	uint16_t Datas_Voltage_Amp_bp;
	uint16_t Datas_Voltage_Amp_ap;
	uint16_t Datas_Current;

	float volts = battery_voltage1 * 2; //in 0.5v resolution
	Datas_Voltage_Amp_bp = volts;
	Datas_Voltage_Amp_ap = (volts - int(volts)) * 100;
	Datas_Current = current_amps1;

	sendDataHead(ID_Voltage_Amp_bp);
	write_FrSky16(Datas_Voltage_Amp_bp);
	sendDataHead(ID_Voltage_Amp_ap);
	write_FrSky16(Datas_Voltage_Amp_ap);
	sendDataHead(ID_Current);
	write_FrSky16(Datas_Current);

}

// Temperature 1
void send_Temperature1(void) {
	sendDataHead(ID_Temprature1);
	write_FrSky16(barometer.get_temperature() / 10);
}

// Temperature 2
void send_Temperature2(void) {
	sendDataHead(ID_Temprature2);
	if (g_gps->status() == GPS::GPS_OK) {
		if (g_gps->fix) {
			write_FrSky16(g_gps->num_sats + 100); // GPS sat count, value > 100 mean 3D Fix
		} else {
			write_FrSky16(g_gps->num_sats); // GPS sat count, value > 100 mean 3D Fix
		}
	} else {
		write_FrSky16(-1);  // GPS disabled
	}
}

// Altitude
void send_Altitude(void) {
	int16_t Datas_altitude_bp;
	uint16_t Datas_altitude_ap;

	float alt = current_loc.alt / 100;

	Datas_altitude_bp = alt;
	Datas_altitude_ap = (alt - int(alt)) * 100;

	sendDataHead(ID_Altitude_bp);
	write_FrSky16(Datas_altitude_bp);
	sendDataHead(ID_Altitude_ap);
	write_FrSky16(Datas_altitude_ap);
}

// RPM
void send_RPM(void) {
	sendDataHead(ID_RPM);
	write_FrSky16(0);
}

// Fuel level 
//only 5 states [0,25,50,75,100]%
void send_Fuel_level(void) {

	uint16_t Datas_Fuel_level;

	//as demo - play with fuel guage
	//Datas_Fuel_level = (cycleCounter % 5) * 25;

	Datas_Fuel_level = 25;  						//1st quater for power
	if (ap.armed) {
		Datas_Fuel_level += 25;               	//2nd for arming 
		if (g_gps->fix) {
			Datas_Fuel_level += 25;    			//3rd for the fix.
			if (ap.home_is_set)
				Datas_Fuel_level += 25;    		//4rd for home set.
		}
	}

	sendDataHead(ID_Fuel_level);
	write_FrSky16(Datas_Fuel_level);
}

// GPS speed
void send_GPS_speed(void) {
	if (g_gps->fix) {
		float gps_speed_ms = g_gps->ground_speed * 0.01;

		sendDataHead(ID_GPS_speed_bp);
		write_FrSky16(gps_speed_ms);
		sendDataHead(ID_GPS_speed_ap);
		write_FrSky16((gps_speed_ms - int(gps_speed_ms)) * 100);
	}
}

// GPS position
void send_GPS_position(void) {
	uint16_t Datas_Longitude_bp;
	uint16_t Datas_Longitude_ap;
	uint16_t Datas_E_W;
	uint16_t Datas_Latitude_bp;
	uint16_t Datas_Latitude_ap;
	uint16_t Datas_N_S;
	float lat = g_gps->latitude / 10000000.0f;
	float lon = g_gps->longitude / 10000000.0f;
	Datas_Longitude_bp = lon;
	Datas_Longitude_ap = (lon - int(lon)) * 10000;
	Datas_E_W = lon < 0 ? 'W' : 'E';
	Datas_Latitude_bp = lat;
	Datas_Latitude_ap = (lat - int(lat)) * 10000;
	Datas_N_S = lat < 0 ? 'S' : 'N';

	sendDataHead(ID_Longitude_bp);
	write_FrSky16(Datas_Longitude_bp);
	sendDataHead(ID_Longitude_ap);
	write_FrSky16(Datas_Longitude_ap);
	sendDataHead(ID_E_W);
	write_FrSky16(Datas_E_W);

	sendDataHead(ID_Latitude_bp);
	write_FrSky16(Datas_Latitude_bp);
	sendDataHead(ID_Latitude_ap);
	write_FrSky16(Datas_Latitude_ap);
	sendDataHead(ID_N_S);
	write_FrSky16(Datas_N_S);
}

// Time
void send_Time(void) {
	uint32_t seconds_since_start = millis() / 1000;

	sendDataHead(ID_Hour_Minute);
	write_FrSky8(uint16_t(seconds_since_start / 3600));
	write_FrSky8(uint16_t((seconds_since_start / 60) % 60));
	sendDataHead(ID_Second);
	write_FrSky16(uint16_t(seconds_since_start % 60));
}

// Course
/*   void send_Course(void)
 {
 uint16_t Datas_Course_bp;
 uint16_t Datas_Course_ap;

 Datas_Course_bp = heading;
 Datas_Course_ap = 0;

 sendDataHead(ID_Course_bp);
 write_FrSky16(Datas_Course_bp);
 sendDataHead(ID_Course_ap);
 write_FrSky16(Datas_Course_ap);
 }*/

// ACC
/*   void send_Accel(void)
 {
 int16_t Datas_Acc_X;
 int16_t Datas_Acc_Y;
 int16_t Datas_Acc_Z;

 Datas_Acc_X = ((float)accSmooth[0] / acc_1G) * 1000;
 Datas_Acc_Y = ((float)accSmooth[1] / acc_1G) * 1000;
 Datas_Acc_Z = ((float)accSmooth[2] / acc_1G) * 1000;

 sendDataHead(ID_Acc_X);
 write_FrSky16(Datas_Acc_X);
 sendDataHead(ID_Acc_Y);
 write_FrSky16(Datas_Acc_Y);
 sendDataHead(ID_Acc_Z);
 write_FrSky16(Datas_Acc_Z);     
 }*/

// GPS altitude
/*   void send_GPS_altitude(void)
 {         
 if (f.GPS_FIX && GPS_numSat >= 4)
 {
 int16_t Datas_GPS_altidute_bp;
 uint16_t Datas_GPS_altidute_ap;

 Datas_GPS_altidute_bp = GPS_altitude;
 Datas_GPS_altidute_ap = 0;

 sendDataHead(ID_GPS_altidute_bp);
 write_FrSky16(Datas_GPS_altidute_bp);
 sendDataHead(ID_GPS_altidute_ap);
 write_FrSky16(Datas_GPS_altidute_ap);
 }
 }*/

// OSD Initialization
void osd_init() {
	Serial1.begin(9600);
}

void osd_heartbeat_50Hz() {

}

void osd_heartbeat_10Hz() {
	if (!LockMe)
		telemetry_frsky();
}

#endif 
