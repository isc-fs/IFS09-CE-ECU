// SPDX-License-Identifier: proprietary
/**
 * @file electronic_diff_can.h
 * @brief CAN interface layer for Bosch LWS, Xsens IMU, and DTI Dual Inverter.
 *
 * Implements:
 * - Parsing of Bosch Motorsport LWS steering wheel angle & speed.
 * - Parsing of Xsens MTi IMU yaw rate & lateral acceleration.
 * - Encoding of DTI Inverter CAN2 commands:
 *     Packet 0x01: Set AC Current (signed int16 Big-Endian * 10 Apk)
 *     Packet 0x0C: Drive Enable (0 or 1)
 * - Decoding of DTI Inverter telemetry:
 *     Packet 0x20: ERPM (int32 BE), Duty, Input Voltage
 *     Packet 0x22: Inverter & Motor Temps, Fault Code
 * - Heartbeat and timeout guards.
 */

#ifndef ELECTRONIC_DIFF_CAN_H_
#define ELECTRONIC_DIFF_CAN_H_

#include <stdint.h>
#include <stdbool.h>
#include "app/differential_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Decoded state from Bosch Motorsport LWS steering sensor.
 */
typedef struct {
    float    steering_angle_deg;       /**< Decoded steering wheel angle in degrees [deg] */
    float    steering_angle_rad;       /**< Decoded steering wheel angle in radians [rad] */
    float    steering_speed_deg_s;     /**< Steering angular velocity [deg/s] */
    uint8_t  status_code;              /**< Sensor health / calibration status code */
    bool     valid;                    /**< True if message healthy and checksum/status ok */
    uint32_t last_rx_timestamp_ms;     /**< System timestamp of last received frame */
} BoschLwsState;

/**
 * @brief Decoded state from Xsens MTi IMU.
 */
typedef struct {
    float    yaw_rate_rads;            /**< Vehicle yaw rate r [rad/s] (positive = turn left / CCW) */
    float    lateral_accel_mps2;       /**< Lateral acceleration a_y [m/s^2] (positive = turn left) */
    float    longitudinal_accel_mps2;  /**< Longitudinal acceleration a_x [m/s^2] */
    bool     valid;                    /**< True if frame healthy and fresh */
    uint32_t last_rx_timestamp_ms;     /**< System timestamp of last received frame */
} XsensImuState;

/**
 * @brief Decoded state from DTI Inverter telemetry.
 */
typedef struct {
    int32_t  erpm;                     /**< Electrical RPM (ERPM) */
    float    mech_rpm;                 /**< Mechanical motor shaft RPM = ERPM / pole_pairs */
    float    wheel_speed_rads;         /**< Wheel angular velocity [rad/s] = (mech_rpm * 2*pi/60) / G */
    float    duty_cycle_pct;           /**< Controller duty cycle [-100.0% to +100.0%] */
    float    input_voltage_v;          /**< Inverter DC-link input voltage [V] */
    float    controller_temp_c;        /**< Inverter semiconductor temperature [degC] */
    float    motor_temp_c;             /**< Motor temperature [degC] */
    uint8_t  fault_code;               /**< Inverter fault code (0 = NO FAULTS) */
    bool     drive_enabled;            /**< Reported drive enable status */
    bool     valid;                    /**< True if telemetry is within timeout */
    uint32_t last_rx_timestamp_ms;     /**< System timestamp of last received frame */
} DtiInverterState;

/**
 * @brief Initializes sensor and actuator interface states.
 */
void electronic_diff_can_init(BoschLwsState* lws,
                             XsensImuState* imu,
                             DtiInverterState* inv_left,
                             DtiInverterState* inv_right);

/**
 * @brief Parses an incoming CAN frame from the vehicle CAN bus.
 *
 * @param cfg       Pointer to CAN configuration.
 * @param can_id    11-bit or 29-bit CAN arbitration identifier.
 * @param is_ext_id True if frame uses extended 29-bit identifier.
 * @param data      Pointer to CAN payload bytes (DLC length).
 * @param dlc       Data length code (0 to 8).
 * @param now_ms    Current system timestamp [ms].
 * @param lws       Pointer to Bosch LWS state to update.
 * @param imu       Pointer to Xsens IMU state to update.
 * @param inv_left  Pointer to Left DTI inverter state to update.
 * @param inv_right Pointer to Right DTI inverter state to update.
 * @return True if frame matched any differential node and was processed.
 */
bool electronic_diff_can_process_rx(const DiffConfig* cfg,
                                    uint32_t can_id,
                                    bool is_ext_id,
                                    const uint8_t* data,
                                    uint8_t dlc,
                                    uint32_t now_ms,
                                    BoschLwsState* lws,
                                    XsensImuState* imu,
                                    DtiInverterState* inv_left,
                                    DtiInverterState* inv_right);

/**
 * @brief Evaluates timeouts for all differential sensors and inverters.
 *
 * @param cfg       Pointer to configuration.
 * @param now_ms    Current system timestamp [ms].
 * @param lws       Pointer to Bosch LWS state.
 * @param imu       Pointer to Xsens IMU state.
 * @param inv_left  Pointer to Left DTI inverter state.
 * @param inv_right Pointer to Right DTI inverter state.
 */
void electronic_diff_can_check_timeouts(const DiffConfig* cfg,
                                        uint32_t now_ms,
                                        BoschLwsState* lws,
                                        XsensImuState* imu,
                                        DtiInverterState* inv_left,
                                        DtiInverterState* inv_right);

/**
 * @brief Builds a DTI Inverter Packet 0x01 (Set AC Current) command frame.
 *
 * Scaled as signed int16 Big-Endian * 10 Apk.
 *
 * @param cfg         Pointer to configuration.
 * @param node_id     Target inverter node ID (e.g. 1 for left, 2 for right).
 * @param current_apk Target peak current in Amperes [Apk].
 * @param out_can_id  Pointer to store calculated CAN ID.
 * @param out_data    Buffer of at least 8 bytes to store message payload.
 * @param out_dlc     Pointer to store DLC (8).
 */
void electronic_diff_can_build_dti_set_current(const DiffConfig* cfg,
                                               uint8_t node_id,
                                               float current_apk,
                                               uint32_t* out_can_id,
                                               uint8_t* out_data,
                                               uint8_t* out_dlc);

/**
 * @brief Builds a DTI Inverter Packet 0x0C (Drive Enable) command frame.
 *
 * @param cfg         Pointer to configuration.
 * @param node_id     Target inverter node ID.
 * @param enable      True = enable drive (1), False = disable drive (0).
 * @param out_can_id  Pointer to store calculated CAN ID.
 * @param out_data    Buffer of at least 8 bytes to store message payload.
 * @param out_dlc     Pointer to store DLC (8).
 */
void electronic_diff_can_build_dti_drive_enable(const DiffConfig* cfg,
                                                uint8_t node_id,
                                                bool enable,
                                                uint32_t* out_can_id,
                                                uint8_t* out_data,
                                                uint8_t* out_dlc);

/**
 * @brief Derives CAN ID for a DTI packet and node ID according to addressing mode.
 *
 * @param cfg       Pointer to configuration.
 * @param packet_id DTI packet ID (e.g. 0x01, 0x0C).
 * @param node_id   Target inverter node ID (1-30 for Standard, 1-254 for Extended).
 * @return Calculated 11-bit or 29-bit CAN ID.
 */
uint32_t electronic_diff_can_make_dti_id(const DiffConfig* cfg, uint8_t packet_id, uint8_t node_id);

/**
 * @brief Extracts packet ID and node ID from a DTI CAN ID according to configuration.
 *
 * @param cfg           Pointer to configuration.
 * @param can_id        Incoming CAN identifier.
 * @param out_packet_id Pointer to store decoded packet ID.
 * @param out_node_id   Pointer to store decoded node ID.
 */
void electronic_diff_can_parse_dti_id(const DiffConfig* cfg,
                                      uint32_t can_id,
                                      uint8_t* out_packet_id,
                                      uint8_t* out_node_id);

#ifdef __cplusplus
}
#endif

#endif /* ELECTRONIC_DIFF_CAN_H_ */
