// SPDX-License-Identifier: proprietary
/**
 * @file differential_config.h
 * @brief Configuration and tuning parameters for IFS-09 Electronic Differential (Torque Vectoring).
 *
 * All parameters are specified in standard SI units (m, kg, s, N, Nm, rad, A, etc.)
 * or milliseconds for timers/timeouts.
 */

#ifndef DIFFERENTIAL_CONFIG_H_
#define DIFFERENTIAL_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mechanical and chassis parameters of the IFS-09 vehicle.
 */
typedef struct {
    float wheelbase_m;               /**< Wheelbase L (front to rear axle) [m] (1.575 m) */
    float track_width_rear_m;        /**< Rear axle track width w [m] (1.200 m) */
    float cg_height_m;               /**< Center of gravity height h_CG [m] (0.28587 m) */
    float dist_cg_front_m;           /**< Longitudinal distance from CG to front axle a [m] (0.78435 m) */
    float dist_cg_rear_m;            /**< Longitudinal distance from CG to rear axle b [m] (0.79065 m) */
    float total_mass_kg;             /**< Total vehicle mass including driver m [kg] (310.0 kg) */
    float yaw_inertia_kgm2;          /**< Vehicle yaw moment of inertia I_z [kg*m^2] (135.0 kg*m^2) */
    float wheel_radius_m;            /**< Dynamic rolling tire radius r_w [m] (0.204 m) */
    float gear_ratio;                /**< Transmission reduction gear ratio G (12.48:1) */
    float steer_ratio;               /**< Steering gear ratio R_steer = (2*pi*L_arm)/C_factor (4.6418) */
    float rack_c_factor_mm_rev;      /**< Steering rack C-factor [mm/rev] (101.52 mm/rev) */
    float steering_arm_m;            /**< Upright steering arm length L_arm [m] (0.075 m) */
    uint8_t motor_pole_pairs;        /**< Number of pole pairs p of the DTI F-MOT (4 pairs) */
    float kt_peak_nm_per_a;          /**< Motor torque constant K_t,pk (peak Nm per peak Ampere, 0.696) */
    float motor_max_torque_nm;       /**< Maximum peak torque per motor [Nm] (31.6 Nm for DTI F-MOT) */
    float cornering_stiff_front;     /**< Front axle cornering stiffness C_f [N/rad] (28000 N/rad) */
    float cornering_stiff_rear;      /**< Rear axle cornering stiffness C_r [N/rad] (32000 N/rad) */
    /** Understeer gradient K_us [rad/(m/s^2)] = (m/L) * (b/C_f - a/C_r) = 0.0007335 */
    float understeer_gradient;       /**< Understeer gradient K_us [rad/(m/s^2)] */
    /** Roll stiffness distribution: K_roll,tras = k_phi,tras / (k_phi,del + k_phi,tras) = 733 / (1013 + 733) = 0.4198167 */
    float rear_roll_stiffness_ratio; /**< Fraction of lateral load transfer taken by rear axle (0.4198) */
    float road_friction_coeff;       /**< Estimated tire-road friction coefficient mu (1.350) */
    bool  enable_party_mode_in_manual; /**< If false, manual driving is restricted to feedforward MingoMode */
    float k_steer_speed_feedforward;   /**< Feedforward gain for steering wheel angular velocity [Nm / (rad/s)] */
    float max_steer_speed_delta_nm;    /**< Maximum torque bias permitted from steering rate feedforward [Nm] */
} DiffVehicleParams;

/**
 * @brief PID controller gains for closed-loop yaw rate feedback (partymode).
 */
typedef struct {
    float kp;                        /**< Proportional gain [Nm / (rad/s)] */
    float ki;                        /**< Integral gain [Nm / rad] */
    float kd;                        /**< Derivative gain [Nm / (rad/s^2)] */
    float i_max_nm;                  /**< Maximum integral anti-windup clamping [Nm] */
    float d_filter_alpha;            /**< Derivative low-pass filter smoothing coefficient (0.0 to 1.0) */
} DiffPidGains;

/**
 * @brief Gain scheduling parameters for kinematic feedforward mode (mingomode).
 *
 * K_diff(delta, v_x) = K_angle(delta) * K_speed(v_x)
 */
typedef struct {
    /* Steering angle zones (angles in radians) */
    float deadband_angle_rad;        /**< Zone 1: Deadband threshold |delta| < 2 deg (par 50/50) */
    float linear_angle_end_rad;      /**< Zone 2: End of linear ramp |delta| <= 22 deg */
    float k_angle_max;               /**< Peak angle gain multiplier at 22 deg */
    float k_angle_sat;               /**< Zone 3: Saturated gain for |delta| > 22 deg */

    /* Speed breakpoints in m/s (0, 15, 30, 42, 62, 82, 100 km/h) */
    float speed_bp_mps[7];           /**< Breakpoints for speed scaling [m/s] */
    float k_speed_bp[7];             /**< Multiplicative factor K_speed corresponding to breakpoints */
} DiffGainTable;

/**
 * @brief Safety limits, thresholds, and virtual LSD / TCS parameters.
 */
typedef struct {
    float max_delta_torque_nm;       /**< Hard clamp on torque differential Delta_T [Nm] */
    float max_slew_rate_nm_s;        /**< Maximum rate of change of motor torque [Nm/s] */
    float beta_crit_rad;             /**< Critical vehicle side-slip angle (spin threshold, ~8.5 deg) [rad] */
    float lsd_slip_threshold;        /**< Virtual LSD trigger threshold (slip deviation as fraction of omega_mean) */
    float lsd_k_slip;                /**< Virtual LSD torque reduction gain per unit slip error */
    float min_velocity_mps;          /**< Minimum speed threshold below which TV is suppressed [m/s] */
    float tcs_slip_threshold;        /**< TCS slip ratio threshold lambda_thresh (0.15 = 15% slip) */
    float tcs_k_slip;                /**< TCS torque derating gain per excess slip ratio */
    float imu_plausibility_diff_mps2;/**< Max difference |a_y - v_x*r| before flagging IMU implausible [m/s^2] */
} DiffLimits;

/**
 * @brief Communication and timeout thresholds.
 */
typedef struct {
    uint32_t imu_timeout_ms;         /**< IMU timeout period [ms] */
    uint32_t steer_timeout_ms;       /**< Bosch LWS steering angle timeout period [ms] */
    uint32_t inverter_timeout_ms;    /**< DTI inverter telemetry timeout period [ms] */
    uint32_t mode_debounce_ms;       /**< Stable IMU reception required to promote mingomode -> partymode [ms] */
} DiffTimeouts;

/**
 * @brief CAN bus IDs and addressing configuration for shared vehicle CAN.
 */
typedef struct {
    /* Bosch Motorsport LWS Sensor */
    uint32_t lws_can_id;             /**< Bosch LWS CAN identifier (Standard 11-bit: 0x2B0 per Bosch LWS datasheet) */

    /* IMU (BMI088 / Xsens) */
    uint32_t imu_can_id;             /**< IMU CAN identifier (Standard 11-bit: 0x201) */

    /* Autonomous / Driverless Node */
    uint32_t udv_can_id;             /**< uDV CAN identifier */

    /* DTI Inverter Addressing */
    bool     dti_use_extended_id;    /**< false = Standard (11-bit: Packet << 5 | Node), true = Extended (29-bit) */
    uint8_t  dti_left_node_id;       /**< Left rear inverter CAN node ID (1) */
    uint8_t  dti_right_node_id;      /**< Right rear inverter CAN node ID (2) */

    /* DTI Packet IDs */
    uint8_t  dti_pkt_set_current;    /**< 0x01: Set AC Current (Signed int16 * 10 Apk) */
    uint8_t  dti_pkt_drive_enable;   /**< 0x0C: Drive Enable (0 or 1) */
    uint8_t  dti_pkt_telemetry_erpm; /**< 0x20: ERPM, Duty, Input Voltage */
    uint8_t  dti_pkt_telemetry_current; /**< 0x21: AC Current, DC Current */
    uint8_t  dti_pkt_telemetry_temp; /**< 0x22: Inverter/Motor Temps, Fault Code */
} DiffCanConfig;

/**
 * @brief Master configuration structure for the Electronic Differential.
 */
typedef struct {
    DiffVehicleParams vehicle;
    DiffPidGains      pid;
    DiffGainTable     gain_table;
    DiffLimits        limits;
    DiffTimeouts      timeouts;
    DiffCanConfig     can;
} DiffConfig;

/**
 * @brief Returns the default production configuration for the IFS-09 vehicle.
 * @return Default DiffConfig instance populated with tested baseline parameters.
 */
DiffConfig diff_config_get_default(void);

#ifdef __cplusplus
}
#endif

#endif /* DIFFERENTIAL_CONFIG_H_ */
