// SPDX-License-Identifier: proprietary
//
// udv_tx -- builders for the ECU -> uDV autonomous-contract frames, the
// pit_diag pattern: pure (no HAL/RTOS), encode via the DSL, return a CanFrame
// for can_tx_post(). Host-testable from the SIL.
//
//   0x504 VCU_ts_active        TS-active flag (the FSM's ok_precharge view)
//   0x505 VCU_brake_over_limit brake sensor >= config::BrakeDvHardRaw (binary)
//   0x506 VCU_motor_rpm        inverter 0x463 rpm relayed (s32 LE) for SLAM

#pragma once

#include "app/can_frame.hpp"

#include <cstdint>

namespace ecu {

class UdvTx {
public:
    static CanFrame build_ts_active(bool ts_active) noexcept;              // 0x504
    static CanFrame build_brake_over_limit(bool over_limit) noexcept;      // 0x505
    static CanFrame build_motor_rpm(std::int32_t erpm) noexcept;           // 0x506 (takes ERPM, sends mechanical rpm = erpm / MotorPolePairs)
    static CanFrame build_r2d_confirm(bool confirmed) noexcept;            // 0x511
};

}  // namespace ecu
