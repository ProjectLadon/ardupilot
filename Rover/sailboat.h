/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
    Rover Sailboat functionality
*/
class Sailboat
{
public:

    // constructor
    Sailboat();

    // enabled
    bool sail_enabled() const { return enable > 0;}

    // true if sailboat navigation (aka tacking) is enabled
    bool tack_enabled() const;

    // setup
    void init();

    // initialise rc input (channel_mainsail)
    void init_rc_in();

    // decode pilot mainsail input and return in steer_out and throttle_out arguments
    // mainsail_out is in the range 0 to 100, defaults to 100 (fully relaxed) if no input configured
    // wingsail_out is in the range -100 to 100, defaults to 0
    // mast_rotation_out is in the range -100 to 100, defaults to 0
    // differential_out is in the range -100 to 100, defaults to 0
    // fore_flap_lim_out is in the range 0 to 100, defaults to 0
    // mizz_flap_lim_out is in the range 0 to 100, defaults to 0
    void get_pilot_desired_mainsail(
        float throttle,
        float steering,
        float &mainsail_out,
        float &wingsail_out,
        float &mast_rotation_out,
        float &differential_out,
        float &fore_flap_lim_out,
        float &mizz_flap_lim_out
    );

    // calculate throttle and mainsail angle required to attain desired speed (in m/s)
    void get_throttle_and_mainsail_out(
        float desired_speed,
        float steering,
        float &throttle_out,
        float &mainsail_out,
        float &wingsail_out,
        float &mast_rotation_out,
        float &differential_out,
        float &fore_flap_out,
        float &mizz_flap_out
    );

    void get_differential(float steering, float &differential_out);

    // calculate steering output for a given point of sail in radians
    void get_point_of_sail_steering(
        float point_of_sail,
        float &steering_out,
        AR_AttitudeControl &attitude_control
    );

    // Velocity Made Good, this is the speed we are traveling towards the desired destination
    float get_VMG() const;

    // handle user initiated tack while in acro mode
    void handle_tack_request_acro();

    // return target heading in radians when tacking (only used in acro)
    float get_tack_heading_rad();

    // handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
    void handle_tack_request_auto();

    // clear tacking state variables
    void clear_tack();

    // returns true if boat is currently tacking
    bool tacking() const;

    // returns true if sailboat should take a indirect navigation route to go upwind
    bool use_indirect_route(float desired_heading_cd) const;

    // calculate the heading to sail on if we cant go upwind
    float calc_heading(float desired_heading_cd);

    // report out the current tack status
    void tack_report(float desired_heading_cd);
    void tack_report(
        float       desired_heading_cd,
        int8_t      current_tack,
        int8_t      new_tack,
        int8_t      should_tack,
        float       tack_request_ms,
        float       crosstrack_error,
        int8_t      currently_tacking,
        float       command_heading
    );

    // calculate the compass heading in radians for a given point of sail
    float calc_point_of_sail_heading_rad(float cos_in, float sin_in);

    // states of USE_MOTOR parameter and motor_state variable
    enum class UseMotor {
        USE_MOTOR_NEVER  = 0,
        USE_MOTOR_ASSIST = 1,
        USE_MOTOR_ALWAYS = 2
    };

    // set state of motor
    // if report_failure is true a message will be sent to all GCSs
    void set_motor_state(UseMotor state, bool report_failure = true);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // return sailboat loiter radius
    float get_loiter_radius() const {return loit_radius;}

private:

    // true if motor is on to assist with slow tack
    bool motor_assist_tack() const;

    // true if motor should be on to assist with low wind
    bool motor_assist_low_wind() const;

    // parameters
    AP_Int8 enable;
    AP_Float sail_angle_min;
    AP_Float sail_angle_max;
    AP_Float sail_angle_ideal;
    AP_Float sail_heel_angle_max;
    AP_Float sail_no_go;
    AP_Float sail_windspeed_min;
    AP_Float xtrack_max;
    AP_Float loit_radius;
    AP_Float diff_steer_mult;
    AP_Float wind_angle_rad;
    AP_Int32 tack_timeout_ms;
    AP_Int32 tack_retry_ms;

    RC_Channel *channel_mainsail;   // rc input channel for controlling mainsail
    RC_Channel *channel_differential;   // rc input channel for controlling sail differential
    RC_Channel *channel_fore_flap;   // rc input channel for controlling foresail flap limits
    RC_Channel *channel_mizz_flap;   // rc input channel for controlling foresail flap limits
    // channels for controlling the point of sail in acro mode
    RC_Channel *channel_sin;
    RC_Channel *channel_cos;
    bool currently_tacking;         // true when sailboat is in the process of tacking to a new heading
    float tack_heading_rad;         // target heading in radians while tacking in either acro or autonomous modes
    uint32_t tack_request_ms;       // system time user requested tack
    uint32_t auto_tack_start_ms;    // system time when tack was started in autonomous mode
    uint32_t tack_clear_ms;         // system time when tack was cleared
    bool tack_assist;               // true if we should use some throttle to assist tack
    UseMotor motor_state;           // current state of motor output
};
