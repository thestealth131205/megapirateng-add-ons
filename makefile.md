# makefile #

The native environment for megapirateNG is the Arduino. But the problem with Arduino is that it has a very simple editor and has some temp folders problems while building.

The original source code from "ArduPilot" project has the other build method implemented but it is made for Linux environment only.

So here comes my solution for Windows users who wants to use better editor (like Eclipse) and/or a lot better build tools (like industry standard "make").



### makefile How-To ###

The newest instructions on configuration for using the makefile are always located inside the makefile itself.

http://code.google.com/p/megapirateng-add-ons/source/browse/trunk/libraries/AP_Common/Arduino.cygwin.mk

After you install & configure as described:

  1. Open command-prompt
  1. Change folder to your current megapirateNG location.
  1. Change folder to the \Arducopter where the main makefile is located.

From here you may issue command like this:
  * 'make configure' - to create configuration script config.mk - you may edit it later to fine tune your builds.
  * 'make' - to actually build your sources
  * 'make clean' - to clean your temp files so next build will be made from the scratch.
  * 'make upload' - to upload your firmware to the board. (define your serial COM port in the config.mk file)