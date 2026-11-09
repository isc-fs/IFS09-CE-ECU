// SPDX-License-Identifier: proprietary
/**
 * @file electronic_diff_can.c
 * @brief Implementation of the CAN interface layer for Bosch LWS, Xsens IMU, and DTI Inverters.
 */

#include "app/electronic_diff_can.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

static int16_t read_be16_signed(const uint8_t* p) {
    return (int16_t)((((uint16_t)p[0]) << 8) | ((uint16_t)p[1]));
}

static int32_t read_be32_signed(const uint8_t* p) {
    return (int32_t)((((uint32_t)p[0]) << 24) |
                     (((uint32_t)p[1]) << 16) |
                     (((uint32_t)p[2]) << 8)  |
                      ((uint32_t)p[3]));
}

void electronic_diff_can_init(BoschLwsState* lws,
                             XsensImuState* imu,
                             DtiInverterState* inv_left,
                             DtiInverterState* inv_right) {
    if (lws)       memset(lws, 0, sizeof(*lws));
    if (imu)       memset(imu, 0, sizeof(*imu));
    if (inv_left)  memset(inv_left, 0, sizeof(*inv_left));
    if (inv_right) memset(inv_right, 0, sizeof(*inv_right));
}

// DTI CAN2 ADDRESSING (DTI Inverter CAN Manual v2.5, Section 2.4):
// - Standard 11-bit IDs: Packet ID in bits 10:5 (shift >> 5), Node ID in bits 4:0 (mask 0x1F).
// - Extended 29-bit IDs: Packet ID in bits 28:8 (shift >> 8), Node ID in bits 7:0 (mask 0xFF).
// Motorola Big-Endian byte order throughout; all frames 8 bytes DLC.
uint32_t electronic_diff_can_make_dti_id(const DiffConfig* cfg, uint8_t packet_id, uint8_t node_id) {
    if (!cfg) return 0;
    if (cfg->can.dti_use_extended_id) {
        /* Extended ID (29 bits): Packet ID << 8 | Node ID */
        return (((uint32_t)packet_id & 0x1FFFFFu) << 8) | ((uint32_t)node_id & 0xFFu);
    } else {
        /* Standard ID (11 bits): Packet ID << 5 | Node ID */
        return (((uint32_t)packet_id & 0x3Fu) << 5) | ((uint32_t)node_id & 0x1Fu);
    }
}

void electronic_diff_can_parse_dti_id(const DiffConfig* cfg,
                                      uint32_t can_id,
                                      uint8_t* out_packet_id,
                                      uint8_t* out_node_id) {
    if (!cfg) return;
    if (cfg->can.dti_use_extended_id) {
        if (out_packet_id) *out_packet_id = (uint8_t)((can_id >> 8) & 0xFFu);
        if (out_node_id)   *out_node_id   = (uint8_t)(can_id & 0xFFu);
    } else {
        if (out_packet_id) *out_packet_id = (uint8_t)((can_id >> 5) & 0x3Fu);
        if (out_node_id)   *out_node_id   = (uint8_t)(can_id & 0x1Fu);
    }
}

// PACKET 0x01: SET AC CURRENT (Manual Section 4.2 Table 10)
// Signed 16-bit Big-Endian @ bytes 0-1, scaled by 10 (0.1 A_pk per LSB).
// Positive current = forward drive (unlike legacy NX/EMC which was negated).
void electronic_diff_can_build_dti_set_current(const DiffConfig* cfg,
                                               uint8_t node_id,
                                               float current_apk,
                                               uint32_t* out_can_id,
                                               uint8_t* out_data,
                                               uint8_t* out_dlc) {
    if (!cfg || !out_can_id || !out_data || !out_dlc) return;

    *out_can_id = electronic_diff_can_make_dti_id(cfg, cfg->can.dti_pkt_set_current, node_id);
    *out_dlc = 8;

    /* Scaled by 10 (0.1 A per LSB), signed int16 Big-Endian */
    float clamped_current = current_apk;
    if (clamped_current > 850.0f)  clamped_current = 850.0f;
    if (clamped_current < -850.0f) clamped_current = -850.0f;

    int16_t scaled = (int16_t)roundf(clamped_current * 10.0f);

    out_data[0] = (uint8_t)((scaled >> 8) & 0xFF);
    out_data[1] = (uint8_t)(scaled & 0xFF);
    out_data[2] = 0x00;
    out_data[3] = 0x00;
    out_data[4] = 0x00;
    out_data[5] = 0x00;
    out_data[6] = 0x00;
    out_data[7] = 0x00;
}

void electronic_diff_can_build_dti_drive_enable(const DiffConfig* cfg,
                                                uint8_t node_id,
                                                bool enable,
                                                uint32_t* out_can_id,
                                                uint8_t* out_data,
                                                uint8_t* out_dlc) {
    if (!cfg || !out_can_id || !out_data || !out_dlc) return;

    *out_can_id = electronic_diff_can_make_dti_id(cfg, cfg->can.dti_pkt_drive_enable, node_id);
    *out_dlc = 8;

    out_data[0] = enable ? 0x01 : 0x00;
    out_data[1] = 0x00;
    out_data[2] = 0x00;
    out_data[3] = 0x00;
    out_data[4] = 0x00;
    out_data[5] = 0x00;
    out_data[6] = 0x00;
    out_data[7] = 0x00;
}

bool electronic_diff_can_process_rx(const DiffConfig* cfg,
                                    uint32_t can_id,
                                    bool is_ext_id,
                                    const uint8_t* data,
                                    uint8_t dlc,
                                    uint32_t now_ms,
                                    BoschLwsState* lws,
                                    XsensImuState* imu,
                                    DtiInverterState* inv_left,
                                    DtiInverterState* inv_right) {
    if (!cfg || !data || dlc == 0) return false;

    /* 1. Bosch Motorsport LWS Steering Wheel Angle Sensor */
    if (lws && (can_id == cfg->can.lws_can_id || can_id == 0x2B0 || can_id == 0x0C0) && dlc >= 4) {
        if (can_id == 0x2B0 || (can_id == cfg->can.lws_can_id && cfg->can.lws_can_id != 0x0C0)) {
            /* Bosch Motorsport LWS standard (Datasheet LWS F02U.V02.894-01):
             * CAN ID: 0x2B0 (LWS_Standard)
             * Byte order: LSB (Intel / Little-Endian)
             * Bytes 0-1: LWS_ANGLE signed int16 LE, 0.1 deg/LSB (+/-780 deg, 0x7FFF = invalid/uncalibrated)
             * Byte 2:    LWS_SPEED uint8, 4.0 deg/s/LSB (0 to 1,016 deg/s, 0xFF = invalid)
             * Byte 3:    bits 0: OK, 1: CAL, 2: TRIM (0x07 = calibrated & valid)
             */
            const int16_t raw_angle = (int16_t)(((uint16_t)data[0]) | (((uint16_t)data[1]) << 8));
            lws->steering_angle_deg = ((float)raw_angle) * 0.1f;
            lws->steering_angle_rad = lws->steering_angle_deg * DEG_TO_RAD;

            const uint8_t raw_speed = data[2];
            lws->steering_speed_deg_s = ((float)raw_speed) * 4.0f;

            lws->status_code = data[3];
            const bool ok = (data[3] & 0x01) != 0;
            const bool cal = (data[3] & 0x02) != 0;
            const bool trim = (data[3] & 0x04) != 0;
            const bool angle_valid = (raw_angle != 0x7FFF);

            lws->valid = ok && cal && trim && angle_valid;
            lws->last_rx_timestamp_ms = now_ms;
            return true;
        } else {
            /* Legacy / fallback format (0x0C0 Big-Endian) */
            const int16_t raw_angle = read_be16_signed(&data[0]);
            lws->steering_angle_deg = ((float)raw_angle) * 0.1f;
            lws->steering_angle_rad = lws->steering_angle_deg * DEG_TO_RAD;

            const int16_t raw_speed = read_be16_signed(&data[2]);
            lws->steering_speed_deg_s = (float)raw_speed;

            lws->status_code = (dlc >= 5) ? data[4] : 0x00;
            lws->valid = true;
            lws->last_rx_timestamp_ms = now_ms;
            return true;
        }
    }

    /* 2. Xsens MTi IMU Sensor */
    if (imu && can_id == cfg->can.imu_can_id && dlc >= 4) {
        /* Bytes 0-1: Yaw Rate (Rate of Turn Z) signed int16 Big-Endian, 0.001 rad/s */
        const int16_t raw_yaw = read_be16_signed(&data[0]);
        imu->yaw_rate_rads = ((float)raw_yaw) * 0.001f;

        /* Bytes 2-3: Lateral Acceleration a_y signed int16 Big-Endian, 0.01 m/s^2 */
        const int16_t raw_ay = read_be16_signed(&data[2]);
        imu->lateral_accel_mps2 = ((float)raw_ay) * 0.01f;

        /* Bytes 4-5: Longitudinal Acceleration a_x (if available) */
        if (dlc >= 6) {
            const int16_t raw_ax = read_be16_signed(&data[4]);
            imu->longitudinal_accel_mps2 = ((float)raw_ax) * 0.01f;
        }

        imu->valid = true;
        imu->last_rx_timestamp_ms = now_ms;
        return true;
    }

    /* 3. DTI Inverter Telemetry Frames */
    uint8_t pkt_id = 0;
    uint8_t node_id = 0;
    electronic_diff_can_parse_dti_id(cfg, can_id, &pkt_id, &node_id);

    DtiInverterState* target_inv = NULL;
    if (node_id == cfg->can.dti_left_node_id) {
        target_inv = inv_left;
    } else if (node_id == cfg->can.dti_right_node_id) {
        target_inv = inv_right;
    }

    if (target_inv != NULL) {
        if (pkt_id == cfg->can.dti_pkt_telemetry_erpm && dlc >= 8) {
            /* Packet 0x20: ERPM (bytes 0-3), Duty (bytes 4-5), Input Voltage (bytes 6-7) */
            target_inv->erpm = read_be32_signed(&data[0]);
            const float p = (cfg->vehicle.motor_pole_pairs > 0) ? (float)cfg->vehicle.motor_pole_pairs : 4.0f;
            target_inv->mech_rpm = ((float)target_inv->erpm) / p;

            const float g = (cfg->vehicle.gear_ratio > 0.01f) ? cfg->vehicle.gear_ratio : 13.0f;
            /* Wheel rad/s = (mech_rpm * 2*pi / 60) / G */
            target_inv->wheel_speed_rads = ((target_inv->mech_rpm * 2.0f * M_PI) / 60.0f) / g;

            const int16_t raw_duty = read_be16_signed(&data[4]);
            target_inv->duty_cycle_pct = ((float)raw_duty) * 0.1f;

            const int16_t raw_vbus = read_be16_signed(&data[6]);
            target_inv->input_voltage_v = (float)raw_vbus;

            target_inv->valid = true;
            target_inv->last_rx_timestamp_ms = now_ms;
            return true;
        } else if (pkt_id == cfg->can.dti_pkt_telemetry_current && dlc >= 4) {
            /* Packet 0x21: AC Current (0-1), DC Current (2-3) */
            const int16_t raw_ac = read_be16_signed(&data[0]);
            target_inv->ac_current_apk = ((float)raw_ac) * 0.1f;

            const int16_t raw_dc = read_be16_signed(&data[2]);
            target_inv->dc_current_a = ((float)raw_dc) * 0.1f;

            target_inv->valid = true;
            target_inv->last_rx_timestamp_ms = now_ms;
            return true;
        } else if (pkt_id == cfg->can.dti_pkt_telemetry_temp && dlc >= 5) {
            /* Packet 0x22: CTLR Temp (0-1), Motor Temp (2-3), Fault Code (4) */
            const int16_t raw_t_ctrl = read_be16_signed(&data[0]);
            target_inv->controller_temp_c = ((float)raw_t_ctrl) * 0.1f;

            const int16_t raw_t_mot = read_be16_signed(&data[2]);
            target_inv->motor_temp_c = ((float)raw_t_mot) * 0.1f;

            target_inv->fault_code = data[4];
            target_inv->valid = true;
            target_inv->last_rx_timestamp_ms = now_ms;
            return true;
        }
    }

    return false;
}

void electronic_diff_can_check_timeouts(const DiffConfig* cfg,
                                        uint32_t now_ms,
                                        BoschLwsState* lws,
                                        XsensImuState* imu,
                                        DtiInverterState* inv_left,
                                        DtiInverterState* inv_right) {
    if (!cfg) return;

    if (lws && lws->valid) {
        if ((now_ms - lws->last_rx_timestamp_ms) > cfg->timeouts.steer_timeout_ms) {
            lws->valid = false;
        }
    }

    if (imu && imu->valid) {
        if ((now_ms - imu->last_rx_timestamp_ms) > cfg->timeouts.imu_timeout_ms) {
            imu->valid = false;
        }
    }

    if (inv_left && inv_left->valid) {
        if ((now_ms - inv_left->last_rx_timestamp_ms) > cfg->timeouts.inverter_timeout_ms) {
            inv_left->valid = false;
        }
    }

    if (inv_right && inv_right->valid) {
        if ((now_ms - inv_right->last_rx_timestamp_ms) > cfg->timeouts.inverter_timeout_ms) {
            inv_right->valid = false;
        }
    }
}
