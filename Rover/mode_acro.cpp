#include "mode.h"
#include "Rover.h"

void ModeAcro::update()
{
    // get speed forward
    //hal.console->printf("Executing acro mode\n");
    //hal.console->printf("===========================\n");
    float speed, desired_steering, desired_throttle, desired_speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // convert pilot stick input into desired steering and throttle
        get_pilot_desired_steering_and_throttle(
            desired_steering, desired_throttle);
        // no valid speed, just use the provided throttle
        g2.motors.set_throttle(desired_throttle);
        //hal.console->printf(
        //    "Executing with throttle %6.2f\t", desired_throttle);
    } else {
        // convert pilot stick input into desired steering and speed
        get_pilot_desired_steering_and_speed(
            desired_steering, desired_speed);
        calc_throttle(desired_speed, true);
        //hal.console->printf(
        //    "Executing with speed %6.2f\t", desired_speed);
    }

    float steering_out, desired_mainsail, desired_wingsail;
    float desired_mast_rotation, desired_differential;
    float desired_fore_flap, desired_mizzen_flap;

    // handle Ladon Robotics sailboats
    float cos_in, sin_in, angle_in;
    get_pilot_desired_roll_and_pitch(sin_in, cos_in);
    angle_in = rover.g2.sailboat.calc_point_of_sail_heading_rad(cos_in, sin_in);
    mavlink_msg_named_value_int_send(MAVLINK_COMM_1, AP_HAL::millis(), "SailHeading", degrees(angle_in));
    rover.g2.sailboat.get_point_of_sail_steering(
	angle_in,
        steering_out,
        attitude_control
    );
    rover.g2.sailboat.get_pilot_desired_mainsail(
        desired_throttle,
        steering_out,
        desired_mainsail,
        desired_wingsail,
        desired_mast_rotation,
        desired_differential,
        desired_fore_flap,
        desired_mizzen_flap
    );

    // handle sailboats
    // if (!is_zero(desired_steering)) {
    //     // steering input return control to user
    //     rover.g2.sailboat.clear_tack();
    // }
    // if (rover.g2.sailboat.tacking()) {
    //     // call heading controller during tacking
    //
    //     steering_out =
    //         attitude_control.get_steering_out_heading(
    //             rover.g2.sailboat.get_tack_heading_rad(),
    //             g2.wp_nav.get_pivot_rate(),
    //             g2.motors.limit.steer_left,
    //             g2.motors.limit.steer_right,
    //             rover.G_Dt);
    // } else {
    //     // convert pilot steering input to desired turn rate in radians/sec
    //     const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
    //
    //     // run steering turn rate controller and throttle controller
    //     steering_out =
    //         attitude_control.get_steering_out_rate(
    //             target_turn_rate,
    //             g2.motors.limit.steer_left,
    //             g2.motors.limit.steer_right,
    //             rover.G_Dt);
    // }

    //hal.console->printf("Sin: %6.2f\tCos: %6.2f\t", sin_in, cos_in);
    //hal.console->printf("Steering Out: %6.2f\t", steering_out);
    //hal.console->printf("\n===========================\n");
    g2.motors.set_steering(steering_out * 4500.0f);
    g2.motors.set_sail_differential(desired_differential);
    g2.motors.set_mainsail(desired_mainsail);
    g2.motors.set_foresail_flap_limit(desired_fore_flap);
    g2.motors.set_mizzen_flap_limit(desired_mizzen_flap);
    calc_and_set_roll_pitch();
}

bool ModeAcro::requires_velocity() const
{
    return !g2.motors.have_skid_steering();
}

// sailboats in acro mode support user manually initiating tacking from transmitter
void ModeAcro::handle_tack_request()
{
    rover.g2.sailboat.handle_tack_request_acro();
}
