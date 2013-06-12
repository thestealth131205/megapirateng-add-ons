// Support for Remzibi OSD
// Based on file OSD_RemzibiV2.pde from WWW.DIYDRONES.COM
// tested with remzibi firmware ardum 1.73 from rcgroup
// coded by sir alex, 
// tested by fr3d on his desktop with flyduino v2 
// only connect TX1 wire to remzibi RX plug! no ground no 5v power ONLY one Wire !
// tested with the poor remzibi gps on  rx/tx2.
// write flight mode (acro, stabilize,etc..) on line 3 and column 1
// write voltage (an0) on line 14 column 1
// write alert voltage on line 13 column 6

//****************************************************************************************
// to run it configure APM_config.h adding thoses lines.
//#define OSD_PROTOCOL OSD_PROTOCOL_REMZIBI
//#define GPS_PROTOCOL GPS_PROTOCOL_NMEA            //for the poor remzibi nmea gps
//#define BATTERY_EVENT  ENABLED                    //enable for checking main voltage in an0 on flyduino
//#define LOW_VOLTAGE			9.9         //min voltage show alarm
//#define VOLT_DIV_RATIO			3.60// with 10k on (+) and 3k9 on(-)
// have fun fr3d
//**************************************************************************************** 

#if OSD_PROTOCOL == OSD_PROTOCOL_REMZIBI

#define SendSer		Serial1.print
#define SendSerln	Serial1.println

/*
byte oldOSDSwitchPosition = 1;

void read_osd_switch()
{
	byte osdSwitchPosition = readOSDSwitch();
	if (oldOSDSwitchPosition != osdSwitchPosition){
		
		switch(osdSwitchPosition)
		{
			case 1: // First position
			set_osd_mode(1);
			break;

			case 2:
			set_osd_mode(0);
			break;
		}

		oldOSDSwitchPosition = osdSwitchPosition;
	}
}

byte readOSDSwitch(void){
  	int osdPulseWidth = APM_RC.InputCh(OSD_MODE_CHANNEL - 1);
	if (osdPulseWidth >= 1450)  return 2;	// Off
	return 1;
}
*/

void osd_init()
{
	Serial1.begin(38400);
	set_osd_mode(1);
}

void osd_heartbeat_10Hz(void)
{
  // keep one of the two following
  // you can leave it in meters and change to imperial units on Remzibi OSD Configuration tool
  double nMult = 0.01; // cm to meters
  //double nMult = 0.032808399; // cm to feets
  
  // Remzibi packet description
  // Supported messages from Ardupilot or any external device must have this format :
  //    2    3   4      5    6    7      8        9      10       11
  //"$A,lat,lng,numSV,[dist],alt,speed,course,[azimuth],gpsDate,gpsTime," CR LF
  //Notice comas must be after message and at end of it 
  //[dist] and [azimuth] can be empty 
  //Lat and lon format as float ex. "-121.123456" 

	SendSer("$A,");
	SendSer((float)current_loc.lat/10000000, 6); //Latitude
	SendSer(",");
	SendSer((float)current_loc.lng/10000000, 6); //Longitude
	SendSer(",");
	SendSer(g_gps->num_sats, DEC); //Satellite Count
	SendSer(",");
        SendSer((int)wp_distance*nMult, DEC); //Distance to Waypoint in m
	SendSer(",");
	// essai baptiste pour avoir la fleche du vario plus précisemment
        SendSer((float)current_loc.alt*nMult, DEC); //Altitude in m
	SendSer(",");
	SendSer((int)g_gps->ground_speed*nMult, DEC); //Ground Speed
	SendSer(",");
	// heading from 0 to 360°
	if(g.compass_enabled) 
	{
    float heading = compass.calculate_heading(ahrs.get_dcm_matrix());
    SendSer(wrap_360(ToDeg(heading) * 100) /100, DEC);
	}
	else
	{
    // TODO: compute heading from GPS
  }
	SendSer(",");
	// Azimuth (optional)
	SendSer(",");
	SendSer(g_gps->date, DEC); //Date
	SendSer(",");
	SendSer(g_gps->time, DEC); //Time
	SendSer(",");
	SendSerln();

# if BATTERY_EVENT == ENABLED
    if(battery_voltage < LOW_VOLTAGE)
    {
      SendSer("$M,6,13,215,215,");     //fr3d colonne 6 ligne 13 
      SendSer("LOW VOLTAGE ALERT");
      SendSer(",");
      SendSerln();                  

      SendSer ("$M,1,14,215,00,");   //fr3d colonne 1 ligne 14 
      SendSer(battery_voltage, 1); 
      SendSer(",");
      SendSerln();
    }
    else
    {
      SendSer ("$M,1,14,213,00,");     //fr3d colonne 1 ligne 14
      SendSer(battery_voltage, 1);
      SendSer(",");
      SendSerln();         
    }
  #endif
  SendSer ("$M,1,4,0,0,"); //fr3d write flight mode column 1 ligne 4
	switch (control_mode)
	{
		case STABILIZE:
			SendSer("Stable          ");
    break;
    
		case ACRO:
			SendSer("Acro            ");
    break;
    
		case ALT_HOLD:
			SendSer("Alt Hold        ");
    break;
    
		case AUTO:
			SendSer("WP:");
      // wp_distance is in cm
      SendSer((int)wp_distance*nMult, DEC);
      SendSer("   ");
    break;
      
		case GUIDED:
			SendSer("Guided          ");
    break;
    
		case LOITER:
			SendSer("Loiter          ");
    break;
    
		case RTL:
			SendSer("RTH:");
      // wp_distance is in cm
      SendSer((int)(wp_distance*nMult), DEC);
      SendSer("   ");
    break;
    
		case CIRCLE:
			SendSer("Circle          ");
    break;
    
		case POSITION:
			SendSer("Position        ");
    break;
	}
	SendSer(",");
  SendSerln();
}

void osd_heartbeat_50Hz(void)
{
  // Horizon artificiel
	SendSer("$I,");
	SendSer(ahrs.roll_sensor/100, DEC); //Roll
	SendSer(",");
	SendSer(ahrs.pitch_sensor/100, DEC); //Pitch
	SendSer(",");
	SendSerln();
} 

void osd_init_home(void)
{
	SendSer("$SH");
	SendSerln();
	SendSer("$CLS");
	SendSerln(); 
	
	// Doing it a second time: trying to prevent remaining caracters glitches after home distance has been initiliazed
	SendSer("$CLS");
	SendSerln(); 
}

void set_osd_mode(int mode)
{
  switch(mode)
  {
    case 1: // On
      SendSerln("$CLS");
      SendSerln("$L1");
    break;

    case 0: // Off
      SendSerln("$L0");
      SendSerln("$CLS");
    break;
  }
}

#endif
