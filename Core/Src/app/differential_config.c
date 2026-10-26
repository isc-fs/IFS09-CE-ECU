// SPDX-License-Identifier: proprietary
/**
 * @file differential_config.c
 * @brief Default parameter definitions for the IFS-09 Electronic Differential.
 */

#include "app/differential_config.h"

DiffConfig diff_config_get_default(void) {
    DiffConfig cfg;

    /* Vehicle physical parameters */
    cfg.vehicle.wheelbase_m               = 1.530f;
    cfg.vehicle.track_width_rear_m        = 1.200f;
    cfg.vehicle.cg_height_m               = 0.280f;
    cfg.vehicle.dist_cg_front_m           = 0.780f;
    cfg.vehicle.dist_cg_rear_m            = 0.750f;
    cfg.vehicle.total_mass_kg             = 260.0f;
    cfg.vehicle.yaw_inertia_kgm2          = 120.0f;
    cfg.vehicle.wheel_radius_m            = 0.203f;
    cfg.vehicle.gear_ratio                = 13.0f;
    cfg.vehicle.steer_ratio               = 4.50f;
    cfg.vehicle.motor_pole_pairs          = 4;
    cfg.vehicle.kt_peak_nm_per_a          = 0.696f;
    cfg.vehicle.motor_max_torque_nm       = 31.6f;
    cfg.vehicle.cornering_stiff_front     = 25000.0f;
    cfg.vehicle.cornering_stiff_rear      = 28000.0f;
    /* K_us = (m/L) * (b/C_f - a/C_r) */
    cfg.vehicle.understeer_gradient       = 0.000364f;
    cfg.vehicle.rear_roll_stiffness_ratio = 0.550f;
    cfg.vehicle.road_friction_coeff       = 1.350f;

    /* PID controller gains */
    cfg.pid.kp                            = 18.0f;
    cfg.pid.ki                            = 3.50f;
    cfg.pid.kd                            = 1.20f;
    cfg.pid.i_max_nm                      = 8.00f;
    cfg.pid.d_filter_alpha                = 0.25f;

    /* Gain scheduling table for mingomode */
    cfg.gain_table.deadband_angle_rad     = 0.03490659f; /* 2.0 deg */
    cfg.gain_table.linear_angle_end_rad   = 0.38397244f; /* 22.0 deg */
    cfg.gain_table.k_angle_max            = 1.0f;
    cfg.gain_table.k_angle_sat            = 0.95f;

    /* Speed breakpoints: 0, 15, 30, 42, 62, 82, 100 km/h in m/s */
    cfg.gain_table.speed_bp_mps[0]        = 0.0f;
    cfg.gain_table.speed_bp_mps[1]        = 4.167f;
    cfg.gain_table.speed_bp_mps[2]        = 8.333f;
    cfg.gain_table.speed_bp_mps[3]        = 11.667f;
    cfg.gain_table.speed_bp_mps[4]        = 17.222f;
    cfg.gain_table.speed_bp_mps[5]        = 22.778f;
    cfg.gain_table.speed_bp_mps[6]        = 27.778f;

    /* Speed factors matching Section 4.2 of IFS-09 specifications */
    cfg.gain_table.k_speed_bp[0]          = 0.0f;
    cfg.gain_table.k_speed_bp[1]          = 0.20f;
    cfg.gain_table.k_speed_bp[2]          = 0.70f;
    cfg.gain_table.k_speed_bp[3]          = 0.90f;
    cfg.gain_table.k_speed_bp[4]          = 1.00f;
    cfg.gain_table.k_speed_bp[5]          = 0.50f;
    cfg.gain_table.k_speed_bp[6]          = 0.15f;

    /* Limits and thresholds */
    cfg.limits.max_delta_torque_nm        = 15.0f;
    cfg.limits.max_slew_rate_nm_s         = 150.0f;
    cfg.limits.beta_crit_rad              = 0.148353f;   /* ~8.5 deg */
    cfg.limits.lsd_slip_threshold         = 0.15f;       /* 15% slip deviation */
    cfg.limits.lsd_k_slip                 = 1.50f;
    cfg.limits.min_velocity_mps           = 0.80f;

    /* Timeout and debouncing */
    cfg.timeouts.imu_timeout_ms           = 50;
    cfg.timeouts.steer_timeout_ms         = 100;
    cfg.timeouts.inverter_timeout_ms      = 100;
    cfg.timeouts.mode_debounce_ms         = 150;

    /* CAN identification and addressing */
    cfg.can.lws_can_id                    = 0x0C0;
    cfg.can.imu_can_id                    = 0x201;
    cfg.can.udv_can_id                    = 0x507;
    cfg.can.dti_use_extended_id           = false;
    cfg.can.dti_left_node_id              = 1;
    cfg.can.dti_right_node_id             = 2;
    cfg.can.dti_pkt_set_current           = 0x01;
    cfg.can.dti_pkt_drive_enable          = 0x0C;
    cfg.can.dti_pkt_telemetry_erpm        = 0x20;
    cfg.can.dti_pkt_telemetry_temp        = 0x22;

    return cfg;
}
