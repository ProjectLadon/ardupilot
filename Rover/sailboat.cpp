#include "Rover.h"

#define SAILBOAT_TACKING_ACCURACY_DEG 10        // tack is considered complete when vehicle is within this many degrees of target tack angle
#define SAILBOAT_NOGO_PAD 10                    // deg, the no go zone is padded by this much when deciding if we should use the Sailboat heading controller
/*
To Do List
 - Improve tacking in light winds and bearing away in strong wings
 - consider drag vs lift sailing differences, ie upwind sail is like wing, dead down wind sail is like parachute
 - max speed parameter and controller, for mapping you may not want to go too fast
 - mavlink sailing messages
 - smart decision making, ie tack on windshifts, what to do if stuck head to wind
 - some sailing codes track waves to try and 'surf' and to allow tacking on a flat bit, not sure if there is much gain to be had here
 - add some sort of pitch monitoring to prevent nose diving in heavy weather
 - pitch PID for hydrofoils
 - more advanced sail control, ie twist
 - independent sheeting for main and jib
 - tack on depth sounder info to stop sailing into shallow water on indirect sailing routes
 - add option to do proper tacks, ie tacking on flat spot in the waves, or only try once at a certain speed, or some better method than just changing the desired heading suddenly
*/

// definitions for TACK dataflash log
#define TACK_LABELS ("TimeUS,DesHdg,CurTck,NuTck,ShldTck,TckReqMS,XtrkErr,CurrTckng,CmdHdg")
#define TACK_UNITS ("sh---sm-h")    // seconds, deg heading, flag, flag,
                                    // flag, flag, meters, flag, deg heading
#define TACK_MULTS ("FB000C00B")    // 1e-6, 1e-2, 1, 1, 1, 1e-3, 1, 1, 1e-2
#define TACK_TYPES ("QfBBBffBf")    // uint64_t, float, uint8_t, uint8_t,
                                    //uint8_t, float, float, uint8_t, float

const AP_Param::GroupInfo Sailboat::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Sailboat
    // @Description: This enables Sailboat functionality
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Sailboat, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ANGLE_MIN
    // @DisplayName: Sail min angle
    // @Description: Mainsheet tight, angle between centerline and boom
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MIN", 2, Sailboat, sail_angle_min, 0),

    // @Param: ANGLE_MAX
    // @DisplayName: Sail max angle
    // @Description: Mainsheet loose, angle between centerline and boom. For direct-control rotating masts, the rotation angle at SERVOx_MAX/_MIN; for rotating masts, this value can exceed 90 degrees if the linkages can physically rotate the mast past that angle.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 3, Sailboat, sail_angle_max, 90),

    // @Param: ANGLE_IDEAL
    // @DisplayName: Sail ideal angle
    // @Description: Ideal angle between sail and apparent wind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_IDEAL", 4, Sailboat, sail_angle_ideal, 25),

    // @Param: HEEL_MAX
    // @DisplayName: Sailing maximum heel angle
    // @Description: When in auto sail trim modes the heel will be limited to this value using PID control
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HEEL_MAX", 5, Sailboat, sail_heel_angle_max, 15),

    // @Param: NO_GO_ANGLE
    // @DisplayName: Sailing no go zone angle
    // @Description: The typical closest angle to the wind the vehicle will sail at. the vehicle will sail at this angle when going upwind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("NO_GO_ANGLE", 6, Sailboat, sail_no_go, 45),

    // @Param: WNDSPD_MIN
    // @DisplayName: Sailboat minimum wind speed to sail in
    // @Description: Sailboat minimum wind speed to continue sail in, at lower wind speeds the sailboat will motor if one is fitted
    // @Units: m/s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("WNDSPD_MIN", 7, Sailboat, sail_windspeed_min, 0),

    // @Param: XTRACK_MAX
    // @DisplayName: Sailing vehicle max cross track error
    // @Description: The sail boat will tack when it reaches this cross track error, defines a corridor of 2 times this value wide, 0 disables
    // @Units: m
    // @Range: 5 25
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("XTRACK_MAX", 8, Sailboat, xtrack_max, 10),

    // @Param: LOIT_RADIUS
    // @DisplayName: Loiter radius
    // @Description: When in sailing modes the vehicle will keep moving within this loiter radius
    // @Units: m
    // @Range: 0 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LOIT_RADIUS", 9, Sailboat, loit_radius, 5),

    // @Param: DIFF_K
    // @DisplayName: Differential steering gain
    // @Description: This controls how much the steering
    // @Units: unitless
    // @Range: 0 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DIFF_K", 10, Sailboat, diff_steer_mult, 100),

    // @Param TACK_TO
    // @DisplayName Tacking timeout (ms)
    // @Description This controls how long the boat will continue to attempt to tack before giving up
    // @Units: ms
    // @Range: 0 1000000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TACK_TO", 11, Sailboat, tack_timeout_ms, 5000),

    // @Param TACK_RTR
    // @DisplayName Tacking retry (ms)
    // @Description This controls how long the boat will wait before retrying a failed tack
    // @Units: ms
    // @Range: 0 1000000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TACK_RTR", 12, Sailboat, tack_retry_ms, 5000),


    AP_GROUPEND
};


/*
  constructor
 */
Sailboat::Sailboat()
{
    AP_Param::setup_object_defaults(this, var_info);
    wind_angle_rad = 0.0f;
}

// true if sailboat navigation (aka tacking) is enabled
bool Sailboat::tack_enabled() const
{
    // tacking disabled if not a sailboat
    if (!sail_enabled()) {
        return false;
    }

    // tacking disabled if motor is always on
    if (motor_state == UseMotor::USE_MOTOR_ALWAYS) {
        return false;
    }

    // disable tacking if motor is available and wind is below cutoff
    if (motor_assist_low_wind()) {
        return false;
    }

    // otherwise tacking is enabled
    return true;
}

void Sailboat::init()
{
    // sailboat defaults
    if (sail_enabled()) {
        rover.g2.crash_angle.set_default(0);
    }

    if (tack_enabled()) {
        rover.g2.loit_type.set_default(1);
    }

    // initialise motor state to USE_MOTOR_ASSIST
    // this will silently fail if there is no motor attached
    set_motor_state(UseMotor::USE_MOTOR_ASSIST, false);
}

// initialise rc inputs (channel_*), may be called intermittently
void Sailboat::init_rc_in()
{
    // get auxiliary throttle value
    RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MAINSAIL);
    if (rc_ptr != nullptr) {
        // use aux as sail input if defined
        channel_mainsail = rc_ptr;
        channel_mainsail->set_angle(100);
        channel_mainsail->set_default_dead_zone(30);
    } else {
        // use throttle channel
        channel_mainsail = rover.channel_throttle;
    }
    // get differential value
    rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::SAIL_DIFF);
    if (rc_ptr != nullptr) {
        // use aux as sail differential input if defined
        channel_differential = rc_ptr;
        channel_differential->set_angle(100);
        channel_differential->set_default_dead_zone(30);
    } else {
        // use lateral channel
	//hal.console->printf("Using lateral channel for sail differential\n");
        channel_differential = rover.channel_lateral;
    }
    // get foresail flap limit
    rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FLAP_LIM_FWD);
    if (rc_ptr != nullptr) {
	//hal.console->printf("Setting fore flap channel\n");
        channel_fore_flap = rc_ptr;
        channel_fore_flap->set_range(100);
        channel_fore_flap->set_default_dead_zone(30);
    } else {
        channel_fore_flap = nullptr;
    }
    // get mizzen flap limit
    rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FLAP_LIM_MIZZ);
    if (rc_ptr != nullptr) {
	//hal.console->printf("Setting mizzen flap channel\n");
        channel_mizz_flap = rc_ptr;
        channel_mizz_flap->set_range(100);
        channel_mizz_flap->set_default_dead_zone(30);
    } else {
        channel_mizz_flap = nullptr;
    }
    // get point of sail sin and cos
    channel_sin = rover.channel_pitch;
    channel_cos = rover.channel_roll;
}

void Sailboat::get_point_of_sail_steering(
    float point_of_sail,
    float &steering_out,
    AR_AttitudeControl &attitude_control
)
{
    steering_out = attitude_control.get_steering_out_heading(
        point_of_sail,
        rover.g2.wp_nav.get_pivot_rate(),
        rover.g2.motors.limit.steer_left,
        rover.g2.motors.limit.steer_right,
        rover.G_Dt
    );
}

// decode pilot mainsail input and return in steer_out and throttle_out arguments
// mainsail_out is in the range 0 to 100, defaults to 100 (fully relaxed) if no input configured
void Sailboat::get_pilot_desired_mainsail(
    float throttle,
    float steering,
    float &mainsail_out,
    float &wingsail_out,
    float &mast_rotation_out,
    float &differential_out,
    float &fore_flap_lim_out,
    float &mizz_flap_lim_out
)
{
	//hal.console->printf("Manual sailboat mode engaged\n");
    // no RC input means mainsail is moved to trim
    if ((rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) || (channel_mainsail == nullptr)) {
        mainsail_out = 100.0f;
        wingsail_out = 0.0f;
        mast_rotation_out = 0.0f;
        differential_out = 0.0f;
        fore_flap_lim_out = 0.0f;
        mizz_flap_lim_out = 0.0f;
        return;
    }
    mainsail_out = constrain_float(throttle, 0.0f, 100.0f);
    wingsail_out = constrain_float(throttle, -100.0f, 100.0f);
    mast_rotation_out = constrain_float(throttle, -100.0f, 100.0f);
    differential_out = constrain_float(steering * diff_steer_mult, -100.0f, 100.0f);
    //hal.console->printf("\nDifferential steering gain is: %6.2f\n", (float)diff_steer_mult);
    if (channel_fore_flap != nullptr) {
        fore_flap_lim_out = constrain_float(channel_fore_flap->get_control_in(), 0.0f, 100.0f);
    } else fore_flap_lim_out = 0.0f;
    if (channel_mizz_flap != nullptr) {
        mizz_flap_lim_out = constrain_float(channel_mizz_flap->get_control_in(), 0.0f, 100.0f);
    } else mizz_flap_lim_out = 0.0f;
}

// calculate throttle and mainsail angle required to attain desired speed (in m/s)
// returns true if successful, false if sailboats not enabled
void Sailboat::get_throttle_and_mainsail_out(
    float desired_speed,
    float steering,
    float &throttle_out,
    float &mainsail_out,
    float &wingsail_out,
    float &mast_rotation_out,
    float &differential_out,
    float &fore_flap_lim_out,
    float &mizz_flap_lim_out
)
{
    if (!sail_enabled()) {
        throttle_out = 0.0f;
        mainsail_out = 0.0f;
        wingsail_out = 0.0f;
        mast_rotation_out = 0.0f;
        differential_out = 0.0f;
        fore_flap_lim_out = 0.0f;
        mizz_flap_lim_out = 0.0f;
        return;
    }

    differential_out = constrain_float(steering * diff_steer_mult, -100.0f, 100.0f);

    // run speed controller if motor is forced on or motor assistance is required for low speeds or tacking
    if ((motor_state == UseMotor::USE_MOTOR_ALWAYS) ||
         motor_assist_tack() ||
         motor_assist_low_wind())
    {
        // run speed controller - duplicate of calls found in mode::calc_throttle();
        throttle_out =
            100.0f * rover.g2.attitude_control.get_throttle_out_speed(
                desired_speed,
                rover.g2.motors.limit.throttle_lower,
                rover.g2.motors.limit.throttle_upper,
                rover.g.speed_cruise,
                rover.g.throttle_cruise * 0.01f,
                rover.G_Dt);
    }
    else
    {
        throttle_out = 0.0f;
    }

    // if we are motoring relax sails
    if (motor_state == UseMotor::USE_MOTOR_ALWAYS) {
        mainsail_out = 100.0f;
        wingsail_out = 0.0f;
        mast_rotation_out = 0.0f;
        return;
    }

    // use PID controller to sheet out, this number is expected approximately in the 0 to 100 range (with default PIDs)
    const float pid_offset =
        rover.g2.attitude_control.get_sail_out_from_heel(
            radians(sail_heel_angle_max), rover.G_Dt) * 100.0f;

    // get apparent wind, + is wind over starboard side, - is wind over port side
    const float wind_dir_apparent =
        degrees(rover.g2.windvane.get_apparent_wind_direction_rad());
    const float wind_dir_apparent_abs = fabsf(wind_dir_apparent);
    const float wind_dir_apparent_sign = is_negative(wind_dir_apparent) ? -1.0f : 1.0f;

    //
    // mainsail control
    //

    // main sails cannot be used to reverse
    if (!is_positive(desired_speed)) {
        mainsail_out = 100.0f;
    } else {
        // Sails are sheeted the same on each side use abs wind direction

        // set the main sail to the ideal angle to the wind
        float mainsail_angle = wind_dir_apparent_abs - sail_angle_ideal;

        // make sure between allowable range
        mainsail_angle = constrain_float(mainsail_angle, sail_angle_min, sail_angle_max);

        // linear interpolate mainsail value (0 to 100) from wind angle mainsail_angle
        float mainsail_base = linear_interpolate(0.0f, 100.0f, mainsail_angle,sail_angle_min,sail_angle_max);

        mainsail_out = constrain_float((mainsail_base + pid_offset), 0.0f ,100.0f);
        fore_flap_lim_out = 100.0f - mainsail_out;
        mizz_flap_lim_out = 100.0f - mainsail_out;
    }

    //
    // wingsail control
    //

    // wing sails auto trim, we only need to reduce power if we are tipping over, must also be trimmed for correct tack
    // dont allow to reduce power to less than 0, ie not backwinding the sail to self-right
    wingsail_out = (100.0f - MIN(pid_offset,100.0f)) * wind_dir_apparent_sign;

    // wing sails can be used to go backwards, probably not recommended though
    if (is_negative(desired_speed)) {
        wingsail_out *= -1.0f;
    }

    //
    // direct mast rotation control
    //

    if (!is_positive(desired_speed)) {
        // rotating sails can be used to reverse, but not in this version
        mast_rotation_out = 0.0f;
    } else {

        if (wind_dir_apparent_abs < sail_angle_ideal) {
            // in irons, center the sail.
            mast_rotation_out = 0.0f;

        } else {

            float mast_rotation_angle;
            if (wind_dir_apparent_abs < (90.0f + sail_angle_ideal)) {
                // use sail as a lift device, at ideal angle of attack, but depower to prevent excessive heel
                // multiply pid_offset by 0.01 to keep the scaling in the same range as the other sail outputs
                // this means the default PIDs should apply reasonably well to all sail types
                mast_rotation_angle = wind_dir_apparent_abs - sail_angle_ideal * MAX(1.0f - pid_offset*0.01f,0.0f);

                // restore sign
                mast_rotation_angle *= wind_dir_apparent_sign;

            } else {
                // use sail as drag device, but avoid wagging the sail as the wind oscillates
                // between 180 and -180 degrees
                mast_rotation_angle = 90.0f;
                if (wind_dir_apparent_abs > 135.0f) {
                    // wind is almost directly behind, keep wing on current tack
                    if (is_negative(SRV_Channels::get_output_scaled(SRV_Channel::k_mast_rotation))) {
                        mast_rotation_angle *= -1.0f;
                    }
                } else {
                    // set the wing on the correct tack, so that is can be sheeted in if required
                    mast_rotation_angle *= wind_dir_apparent_sign;
                }
            }

            // linear interpolate servo displacement (-100 to 100) from mast rotation angle and restore sign
            mast_rotation_out = linear_interpolate(-100.0f, 100.0f, mast_rotation_angle, -sail_angle_max, sail_angle_max);
        }
    }

}

void Sailboat::get_differential(float steering, float &differential_out) {
    differential_out = constrain_float(steering * diff_steer_mult, -100.0f, 100.0f);
}

// Velocity Made Good, this is the speed we are traveling towards the desired destination
// only for logging at this stage
// https://en.wikipedia.org/wiki/Velocity_made_good
float Sailboat::get_VMG() const
{
    // return zero if we don't have a valid speed
    float speed;
    if (!rover.g2.attitude_control.get_forward_speed(speed)) {
        return 0.0f;
    }

    // return speed if not heading towards a waypoint
    if (!rover.control_mode->is_autopilot_mode()) {
        return speed;
    }

    return (speed * cosf(wrap_PI(radians(rover.g2.wp_nav.wp_bearing_cd() * 0.01f) - rover.ahrs.yaw)));
}

// handle user initiated tack while in acro mode
void Sailboat::handle_tack_request_acro()
{
    if (!tack_enabled() || currently_tacking) {
        return;
    }
    // set tacking heading target to the current angle relative to the true wind but on the new tack
    currently_tacking = true;
    tack_heading_rad = wrap_2PI(rover.ahrs.yaw + 2.0f * wrap_PI((rover.g2.windvane.get_true_wind_direction_rad() - rover.ahrs.yaw)));

    tack_request_ms = AP_HAL::millis();
}

// return target heading in radians when tacking (only used in acro)
float Sailboat::get_tack_heading_rad()
{
    if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.yaw)) < radians(SAILBOAT_TACKING_ACCURACY_DEG) ||
       ((AP_HAL::millis() - tack_request_ms) > (uint32_t)tack_timeout_ms)) {
        clear_tack();
    }

    return tack_heading_rad;
}

// handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
void Sailboat::handle_tack_request_auto()
{
    if (!tack_enabled() || currently_tacking) {
        return;
    }

    // record time of request for tack.  This will be processed asynchronously by sailboat_calc_heading
    tack_request_ms = AP_HAL::millis();
}

// clear tacking state variables
void Sailboat::clear_tack()
{
    currently_tacking = false;
    tack_assist = false;
    tack_request_ms = 0;
    tack_clear_ms = AP_HAL::millis();
}

// returns true if boat is currently tacking
bool Sailboat::tacking() const
{
    return tack_enabled() && currently_tacking;
}

// returns true if sailboat should take a indirect navigation route to go upwind
// desired_heading should be in centi-degrees
bool Sailboat::use_indirect_route(float desired_heading_cd) const
{
    if (!tack_enabled()) {
        return false;
    }

    // use sailboat controller until tack is completed
    if (currently_tacking) {
        return true;
    }

    // convert desired heading to radians
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    // check if desired heading is in the no go zone, if it is we can't go direct
    // pad no go zone, this allows use of heading controller rather than L1 when close to the wind
    return fabsf(wrap_PI(rover.g2.windvane.get_true_wind_direction_rad() - desired_heading_rad)) <= radians(sail_no_go + SAILBOAT_NOGO_PAD);
}

// report out the current tack condition
void Sailboat::tack_report(float desired_heading_cd)
{
    tack_report (
        (double)desired_heading_cd,
        -1, -1, -1, -1, (double)-1, -1,
        (double)desired_heading_cd
    );
}

void Sailboat::tack_report(
    float       desired_heading_cd,
    int8_t      curr_tack,
    int8_t      new_tack,
    int8_t      should_tack,
    float       tack_req_ms,
    float       crosstrack_error,
    int8_t      curr_tacking,
    float       command_heading
)
{
    AP::logger().Write(
        // ("TimeUS,DesHeading,CurrTack,NewTack,ShouldTack,TackReq,XtrackErr,
        // CurrTacking,CmdHeading")
        "TACK", TACK_LABELS, TACK_MULTS, TACK_TYPES,
        AP_HAL::micros64(),
        (double)desired_heading_cd,
        curr_tack,
        new_tack,
        should_tack,
        (double)tack_req_ms,
        (double)crosstrack_error,
        curr_tacking,
        (double)command_heading
    );
}

// if we can't sail on the desired heading then we should pick the best heading that we can sail on
// this function assumes the caller has already checked sailboat_use_indirect_route(desired_heading_cd) returned true
float Sailboat::calc_heading(float desired_heading_cd)
{
    if (!tack_enabled()) {
        tack_report(desired_heading_cd);
        return desired_heading_cd;
    }
    bool should_tack = false;

    // find which tack we are on
    const AP_WindVane::Sailboat_Tack current_tack = rover.g2.windvane.get_current_tack();

    // convert desired heading to radians
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    // if the desired heading is outside the no go zone there is no need to change it
    // this allows use of heading controller rather than L1 when desired
    // this is used in the 'SAILBOAT_NOGO_PAD' region
    const float true_wind_rad = rover.g2.windvane.get_true_wind_direction_rad();
    if (fabsf(wrap_PI(true_wind_rad - desired_heading_rad)) > radians(sail_no_go) && !currently_tacking) {

        // calculate the tack the new heading would be on
        const float new_heading_apparent_angle = wrap_PI(true_wind_rad - desired_heading_rad);
        AP_WindVane::Sailboat_Tack new_tack;
        if (is_negative(new_heading_apparent_angle)) {
            new_tack = AP_WindVane::Sailboat_Tack::TACK_PORT;
        } else {
            new_tack = AP_WindVane::Sailboat_Tack::TACK_STARBOARD;
        }

        // if the new tack is not the same as the current tack we need might need to tack
        if (new_tack != current_tack) {
            // see if it would be a tack, the front of the boat going through the wind
            // or a gybe, the back of the boat going through the wind
            const float app_wind_rad = rover.g2.windvane.get_apparent_wind_direction_rad();
            if (fabsf(app_wind_rad) + fabsf(new_heading_apparent_angle) < M_PI) {
                should_tack = true;
            }
        }

        if (!should_tack) {
            tack_report (
                (double)desired_heading_cd,
                (uint8_t)current_tack,
                (uint8_t)new_tack,
                (uint8_t)should_tack,
                (double)tack_request_ms,
                (double)rover.g2.wp_nav.crosstrack_error(),
                (uint8_t)currently_tacking,
                (double)desired_heading_cd
            );
            return desired_heading_cd;
        }
    }

    // check for user requested tack
    uint32_t now = AP_HAL::millis();
    if (tack_request_ms != 0 && !should_tack  && !currently_tacking) {
        // set should_tack flag is user requested tack within last 0.5 sec
        should_tack = ((now - tack_request_ms) < 500);
        tack_request_ms = 0;
    }

    // trigger tack if cross track error larger than xtrack_max parameter
    // this effectively defines a 'corridor' of width 2*xtrack_max that the boat will stay within
    const float cross_track_error = rover.g2.wp_nav.crosstrack_error();
    if ((fabsf(cross_track_error) >= xtrack_max) && !is_zero(xtrack_max) && !should_tack && !currently_tacking) {
        // make sure the new tack will reduce the cross track error
        // if were on starboard tack we are traveling towards the left hand boundary
        if (is_positive(cross_track_error) && (current_tack == AP_WindVane::Sailboat_Tack::TACK_STARBOARD)) {
            should_tack = true;
        }
        // if were on port tack we are traveling towards the right hand boundary
        if (is_negative(cross_track_error) && (current_tack == AP_WindVane::Sailboat_Tack::TACK_PORT)) {
            should_tack = true;
        }
    }

    // calculate left and right no go headings looking upwind, Port tack heading is left no-go, STBD tack is right of no-go
    const float left_no_go_heading_rad = wrap_2PI(true_wind_rad + radians(sail_no_go));
    const float right_no_go_heading_rad = wrap_2PI(true_wind_rad - radians(sail_no_go));

    // if tack triggered, calculate target heading
    if (should_tack && (now - tack_clear_ms) > (uint32_t)tack_retry_ms) {
        gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking");
        // calculate target heading for the new tack
        switch (current_tack) {
            case AP_WindVane::Sailboat_Tack::TACK_PORT:
                tack_heading_rad = right_no_go_heading_rad;
                break;
            case AP_WindVane::Sailboat_Tack::TACK_STARBOARD:
                tack_heading_rad = left_no_go_heading_rad;
                break;
        }
        currently_tacking = true;
        auto_tack_start_ms = now;
    }

    // if we are tacking we maintain the target heading until the tack completes or times out
    if (currently_tacking) {
        // check if we have reached target
        if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.yaw)) <= radians(SAILBOAT_TACKING_ACCURACY_DEG)) {
            clear_tack();
        } else if ((now - auto_tack_start_ms) > (uint32_t)tack_timeout_ms) {
            // tack has taken too long
            if ((motor_state == UseMotor::USE_MOTOR_ASSIST) && (now - auto_tack_start_ms) < (3.0f * (uint32_t)tack_timeout_ms)) {
                // if we have throttle available use it for another two time periods to get the tack done
                tack_assist = true;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking timed out");
                clear_tack();
            }
        }
        // return tack target heading
        tack_report (
            (double)desired_heading_cd,
            (uint8_t)current_tack,
            -1,
            (uint8_t)should_tack,
            (float)tack_request_ms,
            (float)rover.g2.wp_nav.crosstrack_error(),
            (uint8_t)currently_tacking,
            (double)(degrees(tack_heading_rad) * 100.0f)
        );
        return degrees(tack_heading_rad) * 100.0f;
    }

    // return the correct heading for our current tack
    if (current_tack == AP_WindVane::Sailboat_Tack::TACK_PORT) {
        tack_report (
            (double)desired_heading_cd,
            (uint8_t)current_tack,
            -1,
            (uint8_t)should_tack,
            (float)tack_request_ms,
            (float)rover.g2.wp_nav.crosstrack_error(),
            (uint8_t)currently_tacking,
            (double)(degrees(left_no_go_heading_rad) * 100.0f)
        );
        return degrees(left_no_go_heading_rad) * 100.0f;
    } else {
        tack_report (
            (double)desired_heading_cd,
            (uint8_t)current_tack,
            -1,
            (uint8_t)should_tack,
            (float)tack_request_ms,
            (float)rover.g2.wp_nav.crosstrack_error(),
            (uint8_t)currently_tacking,
            (double)(degrees(right_no_go_heading_rad) * 100.0f)
        );
        return degrees(right_no_go_heading_rad) * 100.0f;
    }
}

// Calculated heading, in radians, based on a point of sail
float Sailboat::calc_point_of_sail_heading_rad(float cos_in, float sin_in)
{
    // get the true wind from the windvane
    const float true_wind_rad = rover.g2.windvane.get_true_wind_direction_rad();
    // normalize the sin & cos so that we can feed them in to atan2
    const float norm_factor = 1/((cos_in * cos_in) + (sin_in * sin_in));
    cos_in *= norm_factor;
    sin_in *= norm_factor;
    // if norm_factor is too large, then the stick is centered and we should use the last wind_angle
    if (norm_factor < 10.0f)
    {
        wind_angle_rad = atan2f(sin_in, cos_in);
    }
    float result = wrap_2PI(true_wind_rad - wind_angle_rad);
    //hal.console->printf("Wind (deg): %6.2f\t", true_wind_rad*180/M_PI);
    //hal.console->printf("Target POS (deg): %6.2f\t", (float)wind_angle_rad*180/M_PI);
    //hal.console->printf("Norming: %6.2f\t", norm_factor);
    //hal.console->printf("Target course (deg):%6.2f\n", result*180/M_PI);
    return result;
}

// set state of motor
void Sailboat::set_motor_state(UseMotor state, bool report_failure)
{
    // always allow motor to be disabled
    if (state == UseMotor::USE_MOTOR_NEVER) {
        motor_state = state;
        return;
    }

    // enable assistance or always on if a motor is defined
    if (rover.g2.motors.have_skid_steering() ||
        SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
        rover.get_frame_type() != rover.g2.motors.frame_type::FRAME_TYPE_UNDEFINED) {
        motor_state = state;
    } else if (report_failure) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Sailboat: failed to enable motor");
    }
}

// true if motor is on to assist with slow tack
bool Sailboat::motor_assist_tack() const
{
    // throttle is assist is disabled
    if (motor_state != UseMotor::USE_MOTOR_ASSIST) {
        return false;
    }

    // assist with a tack because it is taking too long
    return tack_assist;
}

// true if motor should be on to assist with low wind
bool Sailboat::motor_assist_low_wind() const
{
    // motor assist is disabled
    if (motor_state != UseMotor::USE_MOTOR_ASSIST) {
        return false;
    }

    // assist if wind speed is below cutoff
    return (is_positive(sail_windspeed_min) &&
            rover.g2.windvane.wind_speed_enabled() &&
            (rover.g2.windvane.get_true_wind_speed() < sail_windspeed_min));
}
