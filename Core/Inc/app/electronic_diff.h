// SPDX-License-Identifier: proprietary
/**
 * @file electronic_diff.h
 * @brief Core Electronic Differential (Torque Vectoring) algorithm for IFS-09.
 *
 * Implements:
 * - State machine handling open-loop (mingomode) and closed-loop (partymode)
 * - Kinematic Ackermann torque split with variable gain scheduling K_diff(delta, v_x)
 * - Virtual LSD via rear wheel ERPM slip feedback
 * - Dynamic 2-DOF bicycle model target yaw rate with friction saturation
 * - Dynamic lateral load transfer base torque distribution
 * - Anti-windup PID yaw rate feedback controller
 * - Sideslip angle (beta) observer and emergency spin stabilization
 * - Strict actuator clipping and net driver torque request preservation
 */

#ifndef ELECTRONIC_DIFF_H_
#define ELECTRONIC_DIFF_H_

#include <stdint.h>
#include <stdbool.h>
#include "app/differential_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Electronic differential operating modes.
 */
typedef enum {
    DIFF_MODE_PASSIVE = 0,    /**< Symmetric 50/50 split (sensor fault, standstill, or safe degraded) */
    DIFF_MODE_MINGO   = 1,    /**< Open-loop kinematic feedforward with Ackermann geometry & Virtual LSD */
    DIFF_MODE_PARTY   = 2     /**< Closed-loop dynamic yaw rate feedback via IMU & 2-DOF bicycle model */
} DiffMode;

/**
 * @brief Diagnostic and status bitmask flags.
 */
#define DIFF_FLAG_IMU_TIMEOUT       (1u << 0) /**< IMU telemetry stale or missing */
#define DIFF_FLAG_STEER_TIMEOUT     (1u << 1) /**< Bosch LWS steering sensor stale or missing */
#define DIFF_FLAG_INVERTER_TIMEOUT  (1u << 2) /**< Inverter telemetry stale */
#define DIFF_FLAG_LSD_ACTIVE        (1u << 3) /**< Virtual LSD slip mitigation actively reducing torque */
#define DIFF_FLAG_YAW_SATURATED     (1u << 4) /**< Target yaw rate saturated by tire friction limit */
#define DIFF_FLAG_TORQUE_CLIPPED    (1u << 5) /**< Output torques clipped to motor/inverter envelope */
#define DIFF_FLAG_EMERGENCY_SPIN    (1u << 6) /**< Side-slip angle exceeded critical limit -> spin recovery */
#define DIFF_FLAG_FALLBACK_ACTIVE   (1u << 7) /**< Operating in degraded fallback mode */

/**
 * @brief Input signals to the Electronic Differential module.
 */
typedef struct {
    float    total_torque_demand_nm;  /**< Validated, rule-compliant driver total torque demand [Nm] */
    float    steering_wheel_angle_rad;/**< Measured steering wheel angle [rad] (positive = turn left) */
    bool     steering_angle_valid;    /**< True if steering sensor data is fresh and healthy */
    float    vehicle_speed_mps;       /**< Vehicle longitudinal velocity v_x [m/s] */
    float    measured_yaw_rate_rads;  /**< Measured vehicle yaw rate r [rad/s] (positive = CCW / turn left) */
    float    lateral_accel_mps2;      /**< Measured lateral acceleration a_y [m/s^2] (positive = turn left) */
    bool     imu_valid;               /**< True if IMU telemetry is fresh and healthy */
    float    left_wheel_speed_rads;   /**< Left rear wheel angular velocity omega_L [rad/s] */
    float    right_wheel_speed_rads;  /**< Right rear wheel angular velocity omega_R [rad/s] */
    bool     wheel_speeds_valid;      /**< True if wheel speed telemetry is valid */
    float    max_motor_torque_limit_nm;/**< External dynamic motor torque limit (e.g. power envelope) [Nm] */
    float    dt_s;                    /**< Timestep duration [s] (e.g. 0.010 for 10 ms) */
    uint32_t now_ms;                  /**< Current system timestamp [ms] */
} DiffInputs;

/**
 * @brief Output commands and diagnostics from the Electronic Differential.
 */
typedef struct {
    float    torque_left_nm;          /**< Commanded shaft torque to left motor [Nm] */
    float    torque_right_nm;         /**< Commanded shaft torque to right motor [Nm] */
    float    current_left_apk;        /**< Commanded peak AC current to left DTI inverter [A] */
    float    current_right_apk;       /**< Commanded peak AC current to right DTI inverter [A] */
    DiffMode mode;                    /**< Currently active control mode */
    float    road_wheel_angle_rad;    /**< Front road wheel steering angle delta = delta_sw / R_steer [rad] */
    float    target_yaw_rate_rads;    /**< Target yaw rate r_target [rad/s] (PartyMode only) */
    float    yaw_rate_error_rads;     /**< Yaw rate tracking error e_r [rad/s] */
    float    yaw_moment_nm;           /**< Commanded corrective yaw moment M_z [Nm] */
    float    delta_torque_nm;         /**< Differential torque bias applied across rear wheels [Nm] */
    float    side_slip_angle_rad;     /**< Estimated vehicle side-slip angle beta [rad] */
    uint16_t diag_flags;              /**< Diagnostic and warning bitmask (DIFF_FLAG_*) */
} DiffOutputs;

/**
 * @brief Internal persistent state of the Electronic Differential.
 */
typedef struct {
    DiffMode mode;                    /**< Current active mode */
    float    last_torque_left_nm;     /**< Previous left torque for slew-rate limiting */
    float    last_torque_right_nm;    /**< Previous right torque for slew-rate limiting */
    float    pid_integral_nm;         /**< Integrated yaw rate error * Ki [Nm] */
    float    last_derivative_nm;      /**< Filtered derivative term */
    float    last_yaw_error_rads;     /**< Previous yaw rate error for derivative calculation */
    float    side_slip_beta_rad;      /**< Integrated vehicle side-slip angle beta [rad] */
    uint32_t imu_healthy_start_ms;    /**< Timestamp when IMU first became healthy during recovery */
    bool     initialized;             /**< Initialization flag */
} DiffState;

/**
 * @brief Initializes the electronic differential state.
 * @param state Pointer to state structure.
 */
void electronic_diff_init(DiffState* state);

/**
 * @brief Executes one periodic step of the Electronic Differential.
 *
 * @param state  Pointer to persistent differential state.
 * @param config Pointer to vehicle configuration parameters.
 * @param in     Pointer to current cycle inputs.
 * @param out    Pointer to output structure to be populated.
 */
void electronic_diff_step(DiffState* state,
                          const DiffConfig* config,
                          const DiffInputs* in,
                          DiffOutputs* out);

/**
 * @brief Evaluates variable differential gain K_diff = K_angle(delta) * K_speed(v_x).
 * @param config Pointer to vehicle configuration parameters.
 * @param delta_rad Road wheel steering angle [rad].
 * @param v_x_mps Vehicle longitudinal velocity [m/s].
 * @return Composite gain K_diff.
 */
float electronic_diff_calc_k_diff(const DiffConfig* config, float delta_rad, float v_x_mps);

/**
 * @brief Evaluates target yaw rate from the 2-DOF linear bicycle model with friction saturation.
 * @param config Pointer to vehicle configuration parameters.
 * @param delta_rad Road wheel steering angle [rad].
 * @param v_x_mps Vehicle longitudinal velocity [m/s].
 * @param saturated_out Optional pointer to bool, set to true if friction limit capped target.
 * @return Saturated target yaw rate [rad/s].
 */
float electronic_diff_calc_target_yaw_rate(const DiffConfig* config,
                                          float delta_rad,
                                          float v_x_mps,
                                          bool* saturated_out);

#ifdef __cplusplus
}
#endif

#endif /* ELECTRONIC_DIFF_H_ */
