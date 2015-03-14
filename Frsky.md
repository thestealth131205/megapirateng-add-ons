# OSD\_FrSky Telemetry module #

Here are the mods for OSD\_FrSky telemetry :

  1. Added functionality of displaying the current state of the copter using Fuel Gauge located on the FrSky LCD screen.
  1. Turnigy 9x/Open9x Voltage/Current sensors integration by HaDa
  1. Compass/GPS heading added to the LCD Display (RPM field)

# How-To #

All instructions are located inside the OSD\_FrSky.pde file.

http://code.google.com/p/megapirateng-add-ons/source/browse/trunk/ArduCopter/OSD_FrSky.pde

### "Copter State" Fuel Gauge ###

You may fine tune this module to work with your mpNG version (2.8/2.9) by un-commenting the lines in the **void send\_Fuel\_level(void)** function.

### Turnigy 9x/Open9x Voltage/Current sensors integration ###

To measure the voltage and the current with your All in One Board (Crius or similar) you need a suitable voltage and current sensor board. On the Crius AIOP V2.0 the voltage input PIN is A0 the current is measured via PIN A1.

Suitable Voltage/Current sensors are the mini AttoPilot modules or the APM 2.5 Power Module. The Module has to be connected between the battery and the ESCs. In case of the "APM 2.5 Power Module" a BEC with a switch power supply is integrated on board which could be used to power the Crius AIOP (do not forget to remove the voltage jumper).

The FrSky telemetry integrates the voltage/current measurement. It could be shown for example on a Turnigy 9x with FrSky-Mod and Open9x installed on it. The display shows voltage and current, the current power uptake in W as an additional information it also shows the used mAh. The Crius AIOP is connected to the telemetry capable receiver via S3 (9600 baud, connect TX-AIOP to RX-FrSky-receiver) but do not forget to add a Level switcher thus the FrSky receiver awaits 3.3V TTL signals on its serial interface (a simple transistor is of use here).

### Compass/GPS heading added to the LCD Display (RPM field) ###

RPM field used for heading data
Two sources to choose from Compass or GPS (review send\_RPM() function to select)
Compass
- take care for compass heading calculations as it takes a lot of the CPU resources
- no declination added so you see raw compass data including all errors !!

GPS (default)
- is much faster but not working on stable copter.
- better solution as a last source of the heading data on compass failure.

LCD Display configuration WARNING:
The value sent are the impulses/second for configured propeller (RPM display is used)
For better accuracy we assume our propeller has 60 blades !!!
(set your blades count to 60 on the LCD config page.)
