#include "Copter.h"

#include "mode.h"

#if MODE_BFACRO_ENABLED == ENABLED

/*
 * Betaflight-style acro mode
 */

float ModeBFAcro::apply_betaflight_expo(const float input, const float expo) const
{
    return input * (1.0f - expo) + input * input * input * expo;
}

void ModeBFAcro::get_pilot_desired_rates(float &roll_out, float &pitch_out, float &yaw_out) const
{
    const float roll_in = channel_roll->norm_input_dz();
    const float pitch_in = channel_pitch->norm_input_dz();
    const float yaw_in = channel_yaw->norm_input_dz();

    // input_rate_bf_roll_pitch_yaw_2 expects rates in centidegrees per second
    roll_out = apply_betaflight_expo(roll_in, g2.command_model_acro_rp.get_expo()) *
               g2.command_model_acro_rp.get_rate() * 100.0f;
    pitch_out = apply_betaflight_expo(pitch_in, g2.command_model_acro_rp.get_expo()) *
                g2.command_model_acro_rp.get_rate() * 100.0f;
    yaw_out = apply_betaflight_expo(yaw_in, g2.command_model_acro_y.get_expo()) *
              g2.command_model_acro_y.get_rate() * 100.0f;
}

void ModeBFAcro::run()
{
    float target_roll;
    float target_pitch;
    float target_yaw;
    get_pilot_desired_rates(target_roll, target_pitch, target_yaw);

    if (!motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero ||
               (copter.air_mode == AirMode::AIRMODE_ENABLED &&
                motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        attitude_control->reset_target_and_rate(true);
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        attitude_control->reset_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        break;
    }

    attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);

    // Direct manual throttle with no angle boost or throttle input filtering
    attitude_control->set_throttle_out(pilot_desired_throttle, false, 0.0f);
}

bool ModeBFAcro::init(bool ignore_checks)
{
    if (g2.acro_options.get() & uint8_t(BFAcroOptions::AIR_MODE)) {
        disable_air_mode_reset = false;
        copter.air_mode = AirMode::AIRMODE_ENABLED;
    }

    attitude_control->reset_rate_controller_I_terms();
    return true;
}

void ModeBFAcro::exit()
{
    if (!disable_air_mode_reset &&
        (g2.acro_options.get() & uint8_t(BFAcroOptions::AIR_MODE))) {
        copter.air_mode = AirMode::AIRMODE_DISABLED;
    }
    disable_air_mode_reset = false;
}

void ModeBFAcro::air_mode_aux_changed()
{
    disable_air_mode_reset = true;
}

float ModeBFAcro::throttle_hover() const
{
    if (g2.acro_thr_mid > 0) {
        return g2.acro_thr_mid;
    }
    return Mode::throttle_hover();
}

#endif
