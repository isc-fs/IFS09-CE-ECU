// SPDX-License-Identifier: proprietary
//
// Plain-data CAN frame used across the FreeRTOS message queues
// (can_rx_queue, can_tx_queue). Kept deliberately POD so it copies cheaply
// and can sit in an osMessageQueue. Encode/decode of specific frame layouts
// lives in per-service code (vehicle_service.cpp, the inverter/pit-diag
// adapters), not here -- this header is just the transport shape.

#pragma once

// The wire layout lives in the C header so freertos.c (CubeMX-generated, C)
// can do sizeof(CanFrame) when constructing the FreeRTOS message queues.
// We re-export the same struct into the ecu:: namespace for the C++ side.
#include "app/can_frame.h"

#include <cstdint>

namespace ecu {

// The two buses on the ECU carrier.
//   Inv = FDCAN1 (PD0/PD1)  -- the NX/EMC inverter (TX setpoints, RX state)
//   Acu = FDCAN2 (PB12/PB13) -- the AMS / accumulator + uDV autonomous bus
// The numeric values match the wire transport byte in CanFrame::bus.
enum class CanBus : std::uint8_t {
    Inv = 0,  // FDCAN1 -- inverter
    Acu = 1,  // FDCAN2 -- AMS / vehicle / autonomous
    Dash = 2, // FDCAN3 -- dashboard / telemetry
};

inline constexpr std::uint8_t CanFrameMaxData = ECU_CAN_FRAME_MAX_DATA;

// Same memory layout as the C struct -- this is just a namespaced alias.
using CanFrame = ::CanFrame;

}  // namespace ecu
