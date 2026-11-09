// SPDX-License-Identifier: proprietary
/**
 * @file electronic_diff.c
 * @brief Core implementation of the IFS-09 Electronic Differential (Torque Vectoring).
 */

#include "app/electronic_diff.h"
#include <math.h>

#define GRAVITY_MPS2 9.80665f

static float clampf(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

void electronic_diff_init(DiffState* state) {
    if (!state) return;
    state->mode                  = DIFF_MODE_PASSIVE;
    state->last_torque_left_nm   = 0.0f;
    state->last_torque_right_nm  = 0.0f;
    state->pid_integral_nm       = 0.0f;
    state->last_derivative_nm    = 0.0f;
    state->last_yaw_error_rads   = 0.0f;
    state->side_slip_beta_rad    = 0.0f;
    state->imu_healthy_start_ms  = 0;
    state->initialized           = true;
}

float electronic_diff_calc_k_diff(const DiffConfig* config, float delta_rad, float v_x_mps) {
    if (!config) return 0.0f;

    const float abs_delta = fabsf(delta_rad);
    float k_angle = 0.0f;

    /* 1. Steering angle dependency (Zone 1, 2, 3) */
    if (abs_delta < config->gain_table.deadband_angle_rad) {
        /* Zone 1: Deadband */
        k_angle = 0.0f;
    } else if (abs_delta <= config->gain_table.linear_angle_end_rad) {
        /* Zone 2: Linear ramp 0 -> k_angle_max */
        const float span = config->gain_table.linear_angle_end_rad - config->gain_table.deadband_angle_rad;
        if (span > 1e-4f) {
            k_angle = ((abs_delta - config->gain_table.deadband_angle_rad) / span) * config->gain_table.k_angle_max;
        } else {
            k_angle = config->gain_table.k_angle_max;
        }
    } else {
        /* Zone 3: Saturated / slight reduction */
        k_angle = config->gain_table.k_angle_sat;
    }

    /* 2. Longitudinal velocity dependency (Ranges 1 to 6) */
    float k_speed = 0.0f;
    const float abs_v = fabsf(v_x_mps);
    const float* bp = config->gain_table.speed_bp_mps;
    const float* k_bp = config->gain_table.k_speed_bp;

    if (abs_v <= bp[0]) {
        k_speed = k_bp[0];
    } else if (abs_v >= bp[6]) {
        k_speed = k_bp[6];
    } else {
        for (int i = 0; i < 6; ++i) {
            if (abs_v >= bp[i] && abs_v <= bp[i + 1]) {
                const float span = bp[i + 1] - bp[i];
                if (span > 1e-4f) {
                    const float frac = (abs_v - bp[i]) / span;
                    k_speed = k_bp[i] + frac * (k_bp[i + 1] - k_bp[i]);
                } else {
                    k_speed = k_bp[i];
                }
                break;
            }
        }
    }

    return k_angle * k_speed;
}

float electronic_diff_calc_target_yaw_rate(const DiffConfig* config,
                                          float delta_rad,
                                          float v_x_mps,
                                          bool* saturated_out) {
    if (saturated_out) *saturated_out = false;
    if (!config) return 0.0f;

    const float abs_v = fabsf(v_x_mps);
    if (abs_v < 0.1f) {
        return 0.0f;
    }

    /* Linear 2-DOF steady-state bicycle model:
     * r_target,lin = [v_x / (L + K_us * v_x^2)] * delta */
    const float denom = config->vehicle.wheelbase_m +
                        config->vehicle.understeer_gradient * abs_v * abs_v;
    if (denom <= 1e-3f) {
        return 0.0f;
    }

    const float r_lin = (v_x_mps / denom) * delta_rad;

    /* Saturation by available road friction limit: |r| <= mu * g / v_x */
    if (abs_v > 0.5f) {
        const float r_fric_max = (config->vehicle.road_friction_coeff * GRAVITY_MPS2) / abs_v;
        if (fabsf(r_lin) > r_fric_max) {
            if (saturated_out) *saturated_out = true;
            return (r_lin >= 0.0f) ? r_fric_max : -r_fric_max;
        }
    }

    return r_lin;
}

void electronic_diff_step(DiffState* state,
                          const DiffConfig* config,
                          const DiffInputs* in,
                          DiffOutputs* out) {
    if (!state || !config || !in || !out) return;

    if (!state->initialized) {
        electronic_diff_init(state);
    }

    /* Clear outputs and diagnostic flags */
    out->torque_left_nm       = 0.0f;
    out->torque_right_nm      = 0.0f;
    out->current_left_apk     = 0.0f;
    out->current_right_apk    = 0.0f;
    out->target_yaw_rate_rads = 0.0f;
    out->yaw_rate_error_rads  = 0.0f;
    out->yaw_moment_nm        = 0.0f;
    out->delta_torque_nm      = 0.0f;
    out->side_slip_angle_rad  = state->side_slip_beta_rad;
    out->slip_ratio_left      = 0.0f;
    out->slip_ratio_right     = 0.0f;
    out->diag_flags           = 0;

    const float dt = (in->dt_s > 1e-4f && in->dt_s < 0.5f) ? in->dt_s : 0.010f;

    /* Base driver torque validation (Formula Student APPS compliance) */
    float t_total = in->total_torque_demand_nm;
    if (t_total < 0.0f) t_total = 0.0f;

    /* 1. Steering angle calculation and safety check.
     * THE STEERING DROPOUT TRAP:
     * Never freeze or hold the last valid steering angle on sensor timeout.
     * When LWS validity drops, fall back to PASSIVE 50/50 immediately. */
    if (!in->steering_angle_valid) {
        out->diag_flags |= DIFF_FLAG_STEER_TIMEOUT;
        state->mode = DIFF_MODE_PASSIVE;
        state->imu_healthy_start_ms = 0;
    }
    const float road_wheel_angle = (config->vehicle.steer_ratio > 0.01f)
        ? (in->steering_wheel_angle_rad / config->vehicle.steer_ratio)
        : 0.0f;
    out->road_wheel_angle_rad = road_wheel_angle;

    /* 2. Sensor Fusion & Plausibility Guards:
     * Kinematic plausibility check on IMU: a_y ~ v_x * r in steady-state.
     * If discrepancy exceeds imu_plausibility_diff_mps2, treat IMU as corrupted. */
    bool imu_plausible = in->imu_valid;
    if (imu_plausible && fabsf(in->vehicle_speed_mps) > 2.0f) {
        const float ay_expected = in->vehicle_speed_mps * in->measured_yaw_rate_rads;
        const float ay_diff = fabsf(in->lateral_accel_mps2 - ay_expected);
        if (ay_diff > config->limits.imu_plausibility_diff_mps2) {
            imu_plausible = false;
            out->diag_flags |= DIFF_FLAG_IMU_IMPLAUSIBLE;
        }
    }

    /* 3. Operating Mode Decision & Bump-less Debouncing:
     * Switching from PartyMode (closed loop) to MingoMode (open loop) is INSTANT.
     * Returning to PartyMode is DEBOUNCED for mode_debounce_ms (150 ms) to avoid chatter. */
    if (in->steering_angle_valid) {
        if (in->imu_valid && imu_plausible) {
            if (state->mode == DIFF_MODE_PARTY) {
                /* Maintain closed-loop mode */
            } else {
                /* Debounce transition to PartyMode */
                if (state->imu_healthy_start_ms == 0) {
                    state->imu_healthy_start_ms = in->now_ms;
                }
                if ((in->now_ms - state->imu_healthy_start_ms) >= config->timeouts.mode_debounce_ms) {
                    state->mode = DIFF_MODE_PARTY;
                } else {
                    state->mode = DIFF_MODE_MINGO;
                    out->diag_flags |= DIFF_FLAG_FALLBACK_ACTIVE;
                }
            }
        } else {
            /* IMU lost, stale, or implausible -> instant bumpless fallback to mingomode */
            state->imu_healthy_start_ms = 0;
            state->mode = DIFF_MODE_MINGO;
            out->diag_flags |= DIFF_FLAG_FALLBACK_ACTIVE;
            if (!in->imu_valid) {
                out->diag_flags |= DIFF_FLAG_IMU_TIMEOUT;
            }
        }
    }
    out->mode = state->mode;

    /* Intermediate raw wheel torques */
    float t_left_raw = 0.0f;
    float t_right_raw = 0.0f;

    /* Cap to maximum allowable motor torque (physical or power-envelope) */
    float motor_cap = config->vehicle.motor_max_torque_nm;
    if (in->max_motor_torque_limit_nm > 0.1f && in->max_motor_torque_limit_nm < motor_cap) {
        motor_cap = in->max_motor_torque_limit_nm;
    }

    /* 4. Compute Torque Distribution According to Mode */
    if (state->mode == DIFF_MODE_PASSIVE || fabsf(in->vehicle_speed_mps) < config->limits.min_velocity_mps) {
        /* Mode 0: Symmetric 50/50 split */
        const float t_base = t_total * 0.5f;
        t_left_raw = t_base;
        t_right_raw = t_base;
        state->pid_integral_nm = 0.0f;
    } else if (state->mode == DIFF_MODE_MINGO) {
        /* Mode 1: Open-loop kinematic Ackermann feedforward + Virtual LSD.
         * Wash out PID integrator to ensure bump-less return to PartyMode */
        state->pid_integral_nm = 0.0f;
        const float t_base = t_total * 0.5f;

        const float k_diff = electronic_diff_calc_k_diff(config, road_wheel_angle, in->vehicle_speed_mps);
        const float tan_delta = tanf(clampf(road_wheel_angle, -0.785f, 0.785f));
        float delta_t = t_base * k_diff * (config->vehicle.track_width_rear_m / (2.0f * config->vehicle.wheelbase_m)) * tan_delta;

        /* Hard clamp to maximum allowable mechanical bias */
        delta_t = clampf(delta_t, -config->limits.max_delta_torque_nm, config->limits.max_delta_torque_nm);
        out->delta_torque_nm = delta_t;

        t_left_raw  = t_base - delta_t;
        t_right_raw = t_base + delta_t;

        /* Virtual LSD with ERPM telemetry feedback */
        if (in->wheel_speeds_valid) {
            const float omega_mean = (in->left_wheel_speed_rads + in->right_wheel_speed_rads) * 0.5f;
            if (omega_mean > 5.0f) {
                const float delta_omega_expected = omega_mean * (config->vehicle.track_width_rear_m / config->vehicle.wheelbase_m) * tan_delta;
                const float delta_omega_measured = in->right_wheel_speed_rads - in->left_wheel_speed_rads;
                const float slip_diff = delta_omega_measured - delta_omega_expected;
                const float slip_thresh = config->limits.lsd_slip_threshold * omega_mean;

                if (fabsf(slip_diff) > slip_thresh) {
                    out->diag_flags |= DIFF_FLAG_LSD_ACTIVE;
                    const float excess_slip = fabsf(slip_diff) - slip_thresh;
                    const float trim = excess_slip * config->limits.lsd_k_slip;

                    if (slip_diff > 0.0f) {
                        t_right_raw -= trim;
                    } else {
                        t_left_raw -= trim;
                    }
                }
            }
        }
    } else if (state->mode == DIFF_MODE_PARTY) {
        /* Mode 2: Closed-loop dynamic yaw rate feedback (Full Torque Vectoring). */
        bool yaw_saturated = false;
        const float r_target = electronic_diff_calc_target_yaw_rate(config, road_wheel_angle, in->vehicle_speed_mps, &yaw_saturated);
        out->target_yaw_rate_rads = r_target;
        if (yaw_saturated) {
            out->diag_flags |= DIFF_FLAG_YAW_SATURATED;
        }

        /* 4.1 Lateral load transfer (Delta F_z)
         * F_z0,tras = m * g * (a / L)
         * Delta_F_z = (m * a_y * h_cg / w) * K_roll,tras */
        const float fz0_rear = config->vehicle.total_mass_kg * GRAVITY_MPS2 *
                               (config->vehicle.dist_cg_front_m / config->vehicle.wheelbase_m);
        const float delta_fz = (config->vehicle.total_mass_kg * in->lateral_accel_mps2 * config->vehicle.cg_height_m /
                                config->vehicle.track_width_rear_m) * config->vehicle.rear_roll_stiffness_ratio;

        /* Clamp vertical load between 5% and 95% of static rear load */
        const float fz_min = 0.05f * fz0_rear;
        const float fz_max = 0.95f * fz0_rear;
        const float fz_left = clampf((fz0_rear * 0.5f) - delta_fz, fz_min, fz_max);
        const float fz_right = clampf((fz0_rear * 0.5f) + delta_fz, fz_min, fz_max);

        /* Base torque allocated proportionally to tire normal load */
        const float t_base_left  = t_total * (fz_left / fz0_rear);
        const float t_base_right = t_total * (fz_right / fz0_rear);

        /* 4.2 Yaw rate error and PID controller with actuator saturation feedback */
        const float e_r = r_target - in->measured_yaw_rate_rads;
        out->yaw_rate_error_rads = e_r;

        /* Proportional term */
        const float p_term = config->pid.kp * e_r;

        /* Integral term with conditional anti-windup clamping:
         * Halt integration if previous motor commands saturated in the direction of error */
        if (t_total > 1.0f) {
            const bool saturated_high = (state->last_torque_left_nm >= motor_cap && e_r < 0.0f) ||
                                        (state->last_torque_right_nm >= motor_cap && e_r > 0.0f);
            const bool saturated_low  = (state->last_torque_left_nm <= 0.0f && e_r > 0.0f) ||
                                        (state->last_torque_right_nm <= 0.0f && e_r < 0.0f);

            if (!saturated_high && !saturated_low) {
                state->pid_integral_nm += config->pid.ki * e_r * dt;
                state->pid_integral_nm = clampf(state->pid_integral_nm, -config->pid.i_max_nm, config->pid.i_max_nm);
            }
        } else {
            /* Reset integral when driver lifts off accelerator */
            state->pid_integral_nm *= 0.90f;
        }
        const float i_term = state->pid_integral_nm;

        /* Derivative term with first-order low-pass filter */
        const float raw_deriv = config->pid.kd * ((e_r - state->last_yaw_error_rads) / dt);
        state->last_yaw_error_rads = e_r;
        const float alpha = config->pid.d_filter_alpha;
        const float d_term = alpha * raw_deriv + (1.0f - alpha) * state->last_derivative_nm;
        state->last_derivative_nm = d_term;

        /* Total corrective yaw moment M_z [Nm] */
        const float m_z = p_term + i_term + d_term;
        out->yaw_moment_nm = m_z;

        /* Convert yaw moment to rear wheel torque differential:
         * Delta_T_TV = (M_z * r_w) / (G * w) */
        float delta_t_tv = (m_z * config->vehicle.wheel_radius_m) /
                           (config->vehicle.gear_ratio * config->vehicle.track_width_rear_m);
        delta_t_tv = clampf(delta_t_tv, -config->limits.max_delta_torque_nm, config->limits.max_delta_torque_nm);
        out->delta_torque_nm = delta_t_tv;

        t_left_raw  = t_base_left - delta_t_tv;
        t_right_raw = t_base_right + delta_t_tv;

        /* 4.3 Vehicle side-slip angle (beta) observer & emergency spin protection */
        if (fabsf(in->vehicle_speed_mps) > 1.5f) {
            const float d_beta = (in->lateral_accel_mps2 / in->vehicle_speed_mps) - in->measured_yaw_rate_rads;
            /* Washout leakage prevents long-term integration drift */
            state->side_slip_beta_rad = state->side_slip_beta_rad * (1.0f - 0.05f * dt) + d_beta * dt;
            out->side_slip_angle_rad = state->side_slip_beta_rad;

            if (fabsf(state->side_slip_beta_rad) > config->limits.beta_crit_rad) {
                out->diag_flags |= DIFF_FLAG_EMERGENCY_SPIN;
                /* Emergency spin stabilization: reduce total torque demand by 70% */
                t_left_raw  *= 0.30f;
                t_right_raw *= 0.30f;
                /* Apply stabilizing counter-moment opposing slip velocity */
                const float counter_bias = 5.0f;
                if (state->side_slip_beta_rad > 0.0f) {
                    t_left_raw  += counter_bias;
                    t_right_raw -= counter_bias;
                } else {
                    t_left_raw  -= counter_bias;
                    t_right_raw += counter_bias;
                }
            }
        }
    }

    /* 5. Active Traction Control (TCS) Slip Ratio Limiter */
    if (in->wheel_speeds_valid && fabsf(in->vehicle_speed_mps) > 0.5f) {
        const float v_ref = (fabsf(in->vehicle_speed_mps) > 1.0f) ? fabsf(in->vehicle_speed_mps) : 1.0f;
        const float v_wheel_l = in->left_wheel_speed_rads * config->vehicle.wheel_radius_m;
        const float v_wheel_r = in->right_wheel_speed_rads * config->vehicle.wheel_radius_m;

        const float slip_l = (v_wheel_l - v_ref) / v_ref;
        const float slip_r = (v_wheel_r - v_ref) / v_ref;

        out->slip_ratio_left  = slip_l;
        out->slip_ratio_right = slip_r;

        const float slip_limit = config->limits.tcs_slip_threshold;
        if (slip_l > slip_limit) {
            out->diag_flags |= DIFF_FLAG_TCS_ACTIVE;
            const float excess = slip_l - slip_limit;
            const float cut_factor = clampf(1.0f - excess * config->limits.tcs_k_slip, 0.0f, 1.0f);
            t_left_raw *= cut_factor;
        }
        if (slip_r > slip_limit) {
            out->diag_flags |= DIFF_FLAG_TCS_ACTIVE;
            const float excess = slip_r - slip_limit;
            const float cut_factor = clampf(1.0f - excess * config->limits.tcs_k_slip, 0.0f, 1.0f);
            t_right_raw *= cut_factor;
        }
    }

    /* 6. Asymmetric Inverter Fault Protection:
     * If either inverter trips, times out, or reports a fault, immediately cut BOTH
     * motors symmetrically to zero to prevent catastrophic uncommanded yaw moments. */
    if (in->inv_left_fault || in->inv_right_fault) {
        t_left_raw  = 0.0f;
        t_right_raw = 0.0f;
        state->pid_integral_nm = 0.0f;
        out->diag_flags |= DIFF_FLAG_ASYMMETRIC_FAULT;
    }

    /* 7. Actuator Envelope & Net Driver Torque Preservation */
    if (t_left_raw < 0.0f)  t_left_raw = 0.0f;
    if (t_right_raw < 0.0f) t_right_raw = 0.0f;

    if (t_left_raw > motor_cap || t_right_raw > motor_cap) {
        out->diag_flags |= DIFF_FLAG_TORQUE_CLIPPED;
    }
    t_left_raw  = clampf(t_left_raw, 0.0f, motor_cap);
    t_right_raw = clampf(t_right_raw, 0.0f, motor_cap);

    /* Invariant: Net delivered torque must never exceed validated driver demand */
    const float raw_sum = t_left_raw + t_right_raw;
    if (raw_sum > t_total && raw_sum > 1e-3f) {
        const float scale = t_total / raw_sum;
        t_left_raw  *= scale;
        t_right_raw *= scale;
    }

    /* 8. Slew-rate limiting for bump-less transitions:
     * Prevents abrupt torque steps during mode degradation or driver transitions.
     * EXCEPTION: Asymmetric inverter faults bypass slew-rate limiting and cut torque
     * immediately (0 ms lag) to eliminate uncommanded yaw spin moments. */
    if (out->diag_flags & DIFF_FLAG_ASYMMETRIC_FAULT) {
        out->torque_left_nm  = 0.0f;
        out->torque_right_nm = 0.0f;
        state->last_torque_left_nm  = 0.0f;
        state->last_torque_right_nm = 0.0f;
    } else {
        const float max_step_up = config->limits.max_slew_rate_nm_s * dt;
        const float max_step_down = (out->diag_flags & DIFF_FLAG_EMERGENCY_SPIN)
            ? (config->limits.max_slew_rate_nm_s * dt * 8.0f)
            : (config->limits.max_slew_rate_nm_s * dt * 2.5f);

        const float d_left  = clampf(t_left_raw - state->last_torque_left_nm, -max_step_down, max_step_up);
        const float d_right = clampf(t_right_raw - state->last_torque_right_nm, -max_step_down, max_step_up);

        out->torque_left_nm  = state->last_torque_left_nm + d_left;
        out->torque_right_nm = state->last_torque_right_nm + d_right;

        state->last_torque_left_nm  = out->torque_left_nm;
        state->last_torque_right_nm = out->torque_right_nm;
    }

    /* 9. Convert torque to peak AC current for DTI Inverter: I_pk = T / K_t,pk */
    if (config->vehicle.kt_peak_nm_per_a > 1e-4f) {
        out->current_left_apk  = out->torque_left_nm / config->vehicle.kt_peak_nm_per_a;
        out->current_right_apk = out->torque_right_nm / config->vehicle.kt_peak_nm_per_a;
    }
}
