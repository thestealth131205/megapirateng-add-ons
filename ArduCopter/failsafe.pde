// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
//  failsafe support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool failsafe_enabled = true;
static uint16_t failsafe_last_mainLoop_count;
static uint32_t failsafe_last_timestamp;

static uint32_t failsafe_max_timestamp;
static uint32_t failsafe_disarm_counter;
static uint32_t failsafe_call_counter;

static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//
void failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
    failsafe_max_timestamp = 0;
    failsafe_disarm_counter = 0;
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.
//
void failsafe_check(uint32_t tnow)
{
	
	uint32_t dtnow = 0;
	
    if (mainLoop_count != failsafe_last_mainLoop_count) {
        // the main loop is running, all is OK
        failsafe_last_mainLoop_count = mainLoop_count;
        failsafe_last_timestamp = tnow;
        in_failsafe = false;
        return;
    } else
    if (tnow < failsafe_last_timestamp)
    	return;
    else    	
		dtnow = tnow -failsafe_last_timestamp;    	

#ifdef CLI_DEBUG	
	failsafe_call_counter++;
	if(failsafe_max_timestamp < dtnow)
    	failsafe_max_timestamp = dtnow;
#endif     

    if (failsafe_enabled && dtnow > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
    }

    if (failsafe_enabled && in_failsafe && dtnow > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
#ifdef CLI_DEBUG        
        failsafe_disarm_counter++;
#endif
        if(motors.armed()) {
            motors.armed(false);
        	set_armed(true);
            motors.output();
            Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE, ERROR_CODE_FAILSAFE_WATCHDOG);
        }
    }
}

uint32_t get_failsafe_max_timestamp()
{
	uint32_t temp=failsafe_max_timestamp;
	failsafe_max_timestamp=0;
    return temp;
}

uint32_t get_failsafe_disarm_counter()
{
    return failsafe_disarm_counter;
}


uint32_t get_failsafe_call_counter()
{

	uint32_t temp=failsafe_call_counter;
	failsafe_call_counter=0;
    return temp;
}

