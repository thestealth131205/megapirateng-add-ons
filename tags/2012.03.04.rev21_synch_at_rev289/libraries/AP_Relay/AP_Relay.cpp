// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * AP_Relay.cpp
 *
 *  Created on: Oct 2, 2011
 *      Author: Amilcar Lucas
 */

#include <AP_Common.h>
#include <DigitalWriteFast.h>

#include "AP_Relay.h"

void AP_Relay::on()
{
	digitalWriteFast(Relay_Pin,1);
}


void AP_Relay::off()
{
	digitalWriteFast(Relay_Pin,0);
}


void AP_Relay::toggle()
{
	if (digitalReadFast(Relay_Pin) == 0)
		on();
	else
		off();
}


void AP_Relay::set(bool status)
{
	if (status)
		on();
	else
		off();
}


bool AP_Relay::get()
{
	return digitalReadFast(Relay_Pin);
}
