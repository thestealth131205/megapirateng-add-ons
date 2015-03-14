# Sonar disable #

Sonar mod is a way to fully disable CPU work on sonar interrupts and functions, making CPU life easier this way.
Before the mod there are still Sonar joint code running all the time even if you are not using Sonar at all.

But there is one important factor:

> Sonar code is partially used for Crius v2 (LP1) PPM code and all PWM (Rx connected by more then 1 cable) Rx modes.
> In other words this mod will work for you ONLY if you use PPM\_SUM mode on pin A8.



Look here to see how it's done:

http://code.google.com/p/megapirateng-add-ons/source/diff?spec=svn70&r=70&format=side&path=/branches/PPM/ArduCopter/APM_Config.h

and here (to disable tests for sonar)

http://code.google.com/p/megapirateng-add-ons/source/diff?spec=svn70&r=70&format=side&path=/branches/PPM/ArduCopter/test.pde


