// Support for HobbyKing E-OSD with Syberian's firmware

#if OSD_PROTOCOL == OSD_PROTOCOL_SYBERIAN

#define Sport		Serial1

// Global vars
byte osd_pak=0;
byte osd_sync=0xcb;

// OSD Initialization
void osd_init()
{
	Sport.begin(9600);
}


/*buffer format:
0-sync word changed (0xcb, 0x34)
1  packet type (0 - ahi,bat,cur,mah, 1 - nav, 2 - maintenance
2       0               1               2
3       roll            gpslat1         gpsSPD
4       pitch           gpslat2         gpsALTH
5                       gpslat3         gpaALTL
6       homeL           gpslat4         gpsDISH
7       control_mode    gpslon1         gpsDISL
8       BATl            gpslon2         status
9       Ih              gpslon3         config
10      IL (up 80.0A)   gpslon4         emerg

==
status:
0..3
        0-Manual
*/


void osd_heartbeat_50Hz() // send each packet via
{
	byte j=0;
	int dcmr,dcmp,hom;

	osd_pak++;
	if (osd_pak>2) osd_pak=0; // from 0 to 3
		
	switch(osd_pak)
	{ 
	 case 0:  
					dcmr=ahrs.roll_sensor*0.005;
					
					dcmp=ahrs.pitch_sensor*0.01;
					if(dcmp<-31) dcmp=-31;
					if(dcmp>31) dcmp=31;
					hom=get_bearing_cd(&current_loc,&home);
					// 270 = 0
					// 0 = 31
					// 90 = 63
					if(hom<9000) hom=32+hom*0.003555;
					else hom=(hom-27000)*0.003555;
					if (hom>63)hom=63;
					if(hom<0) hom=0;
					
					
					Sport.write(osd_sync);
					Sport.write(osd_pak);
					Sport.write(osd_pak);
					Sport.write((char)dcmr);
					Sport.write((char)dcmp);
					spp_writes(hom);  
					Sport.write((byte)control_mode);  // voltage is unused - FREEEEEE 2 bytes
					Sport.write(j);  
					Sport.write(j);  
					Sport.write(j);  
					 
					break;
   case 1:
					// GPS coords (2x4b Long with 4 digs after dot
					Sport.write(osd_sync);
					Sport.write(osd_pak);
					Sport.write(osd_pak);
					spp_writel((long)current_loc.lat*0.001);  
					spp_writel((long)current_loc.lng*0.001);  
					  
					break;
					  case 2:
					  int alt_tmp=(current_loc.alt - home.alt)/100;
					  if(alt_tmp<0) alt_tmp=0;
					Sport.write(osd_sync);
					Sport.write(osd_pak);
					Sport.write(osd_pak);
					Sport.write((char)g_gps->ground_speed/100);
					spp_writes(alt_tmp);  
					spp_writes(get_distance(&current_loc,&home));
					Sport.write((byte)g_gps->num_sats) ;//
					Sport.write(1);
					j=0; // count emergency signals
					if(APM_RC.GetState()!=1) j|=2; // radio failure
					Sport.write(j); //write state
					break;
	}
	// end txing packet
	osd_sync^=255;
}

void spp_writel(long in)
{
	Sport.write((char)(in>>24));  
	Sport.write((char)(in>>16));  
	Sport.write((char)(in>>8));  
	Sport.write((char)(in&255));  
}
void spp_writes(int in)
{
	Sport.write((char)(in>>8));  
	Sport.write((char)(in&255));  
}

void osd_heartbeat_10Hz()
{
	// all work done at 50Hz
}

#endif