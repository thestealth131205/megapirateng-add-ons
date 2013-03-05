Release Notes for MegaPirateNG 2.9 R6 (ArduCopter 2.9.1)

=== How to compile MegaPirateNG ===

1. Download and install Arduino 1.0.1 - 1.0.3!!!!
2. See additional requirements for BlackVortex boards in the release_notes_BlackVortex.txt
3. Delete original libraries folder in Arduino IDE folder
4. Copy libraries folder from MegaPirateNG distributive into Arduino IDE folder
5. Select your Frame, Sensor, Tranmitter type in the APM_config.h (Quad X, ALLINONE by default)

=== Arduino board pin mapping ===

*** RC channels (can be changed in APM_RC_PIRATES.cpp) ***
* PPM SUM signal must be connected to A8 pin
* Default is TX_mwi - MultiWii set
A8	- ROLL
A9	- THROTTLE
A10	- PITCH
A11	- YAW
A12	- AUX1
A13	- AUX2
A14	- CAMPITCH
A15	- CAMROLL

A4-A7 - Copter LEDs. Connect your LED strips to this pins via ULN2003
A0 - Voltage sensor pin
A1 - Current sensor pin

*** Camera stabilization ***
d44 - Camera stabilisation Roll servo
d45 - Camera stabilisation Pitch servo
d46 - Camera trigger relay (5V - On, 0v - Off)

For BlackVortex:
d32 - Camera stabilisation Roll servo
d33 - Camera stabilisation Pitch servo

Serial ports:
Serial0 (RX0,TX0) - USB/Console/Mavlink
Serial1 (RX1,TX1) - OSD (Remzibi, E-OSD, FrSky) 
Serial2 (RX2,TX2) - GPS
Serial3 (RX3,TX3) - Telemetry (3DR, Xbee, Bluetooth)

*** Sonar ***
d9 - Sonar Tx 
d10 - Sonar Echo 

*** Status LEDs ***
d13 - RED
d30 - YELLOW
d31 - GREEN

*** Motor mapping ***
motor mapping (maximize your text viewer or disable word wrapping to avoid line breaks)
======================================================
Pin     D2    D3    D5    D6    D7    D8    D11   D12   - Arduino pins
CH      3     4     1     2     7     8     10    11    - MegaPirate output channel
MOT_*   3     4     1     2     5     6     7     8     - Arducopter motor mixer mapping
TIMER   3     3     3     4     4     4     1     1     - ATMEGA Timer used to generate PWM signal
======================================================
TRI     LC    BC    RC    -     S     -     -     -
QuadX   LFW   RBW   RFC   LBC   -     -     -     -
Quad+   FW    BW    RC    LC    -     -     -     -
Hexa+   BLW   FRC   FW    BC    FLC   BRW   -     -
HexaX   FLW   BRC   RW    LC    FRC   BLW   -     -
Y6      LDC   BDW   RDC   LUW   RUW   BUC   -     -
Octo+   FRC   BRC   FW    BW    FLC   BLC   LW    RW
OctoX   RFC   BRC   FRW   BLW   FLC   LBC   LFW   RBW
OctoV   BLC   BBRC  FLW   BRW   FFLC  FRC   FFRW  BBLW
Quad8X  BLUC  BRUW  FRUC  FLUW  FLDC  FRDW  BRDC  BLDW
Quad8+  BUC   RUW   FUC   LUW   LDC   FDW   RDC   BDW     <<< Not verified
======================================================

Motors description:
B- back
R- right
L- left
F- front
U- upper
D- lower
W- clockwise rotation
C- counter clockwise rotation (normal propeller)
S- servo (for tri)

Example: FLDW - front-left lower motor with clockwise rotation (Y6 or Y4)


=== MPNG History ===
--- 2.9 R6
Changed motor mapping for Tri. Now Motors connected to D2,D3,D5 and servo to D7
Fixed some bugs in PPM Decoder (spikes in channel readings)
Added Jitter and Average filter to PPM SUM decoder
Set 20Hz LPF for MPU6050 by default

--- 2.8 R4
FAILSAFE support for receivers without it (Trigger on signal lost (When receiver stops to send signal to the board)

--- 2.8 R3
AHRS_GPS_GAIN disabled
Added support for PPM_SUM on PL1 pin (for CRIUS AIOP v2)

--- 2.8 R2
Added support for FrSky telemetry
updated motor mapping table in the README.txt

--- 2.7.1 R5
Added support for CRIUS AIOP v2

--- 2.6 R6
Added support for CRIUS All In One PRO v1 board

--- 2.6 R4
Improved I2c_spy.pde to catch 6050 I2c slave devices and also 5611 baro
Fixed bug in compass initialization for boards with MPU6050
Fixed bug in AP_GPS_NMEA

--- 2.6 R3
Fixed GPS initialization for BlackVortext boards

--- 2.6 R2
Added support for Drotek 10DOF IMU

--- 2.5.1 R3
Fixed compass initialization for MPU6050+HMC5883L
MT3329 initialization fixed in case of Cold start

--- 2.5.1 R2
Fixed compilation bug in BMP085 library

--- 2.5 R2
Fixed compass error
Fixed define redeclaration in APM_Config.h

--- 2.4.1 R2
Restored FastPWM, 400Hz by default, can be changed in APM Planner, RC_SPEED value
GPS Initialization fixed, at least my MTK16 protocol now work fine
Stability fix for InstantPWM

--- 2.4.1
Fixed bug in FFIMU sensor board, thanks to Thomas(thnilsen)
Fixed LED Sequencer initialization code, thanks to Thomas(thnilsen)

--- 2.3 rc1
TX configuration moved from APM_RC_PIRATES.cpp to common configuration file APM_config.h

--- 2.3
Version is compatible with Arduino IDE 1.0!
