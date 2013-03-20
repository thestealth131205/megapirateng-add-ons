#define THISFIRMWARE "MegaPirateNG Add-Ons V2.9 R7.$WCREV$ $WCNOW=%Y/%m/%d$"

/*
void AP_MotorsMatrix::output_disarmed()
{


    if (failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
    }



    if(_rc_throttle->control_in > 0) {
        // we have pushed up the throttle
        // remove safety for auto pilot
        _auto_armed = true;
    }

    // Send minimum values to all motors
    output_min();
}


    APM_RC_Class*       _rc;                            // APM_RC class used to send updates to ESCs/Servos
    RC_Channel*         _rc_roll, *_rc_pitch, *_rc_throttle, *_rc_yaw;  // input in from users
    uint8_t             _motor_to_channel_map[AP_MOTORS_MAX_NUM_MOTORS];        // mapping of motor number (as received from upper APM code) to RC channel output - used to account for differences between APM1 and APM2
    uint16_t            _speed_hz;                      // speed in hz to send updates to motors
    bool                _armed;                         // true if motors are armed
    bool                _auto_armed;            // true is throttle is above zero, allows auto pilot to take control of throttle
    uint8_t             _frame_orientation;     // PLUS_FRAME 0, X_FRAME 1, V_FRAME 2
    int16_t             _min_throttle;          // the minimum throttle to be sent to the engines when they're on (prevents issues with some motors on while other off at very low throttle)
    int16_t             _max_throttle;          // the minimum throttle to be sent to the engines when they're on (prevents issues with some motors on while other off at very low throttle)
    AP_CurveInt16_Size4 _throttle_curve;                // curve used to linearize the pwm->thrust
    AP_Int8             _throttle_curve_enabled;        // enable throttle curve
    AP_Int8             _throttle_curve_mid;  // throttle which produces 1/2 the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
    AP_Int8             _throttle_curve_max;  // throttle which produces the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
    uint8_t             _reached_limit;                // bit mask to record which motor limits we hit (if any) during most recent output.  Used to provide feedback to attitude controllers
};


*/
