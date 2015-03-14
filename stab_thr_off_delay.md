# Throttle Off Delay #

As you may know there is a possibility to stop motors in the air while flying STAB or ACRO mode.
This mod delays sending OFF command to the motors by a given time, so if you move your Throttle Stick to the full bottom the motors will not stop immediately.


Delay defined in APM\_Config.h (default 25/50sec )

`#define STAB_THR_OFF_DELAY 25`

http://code.google.com/p/megapirateng-add-ons/source/browse/trunk/ArduCopter/APM_Config.h?spec=svn104&r=104#99

How it's done inside the code (line 1937):

http://code.google.com/p/megapirateng-add-ons/source/diff?spec=svn104&r=104&format=side&path=/trunk/ArduCopter/ArduCopter.pde