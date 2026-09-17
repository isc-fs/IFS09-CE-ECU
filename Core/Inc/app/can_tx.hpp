// SPDX-License-Identifier: proprietary
//
// The one way app code sends a CAN frame: post it to can_tx_queue and let
// CanTxTask drain it to the HAL (single TX point). Non-blocking + safe from any
// task, so the realtime ControlTask never blocks on a full TX FIFO.

#pragma once

#include "app/can_frame.hpp"

namespace ecu {

// Enqueue a frame for CanTxTask. Non-blocking; returns false if the queue is
// full or not yet created. The frame's `bus` field selects FDCAN1/INV vs
// FDCAN2/ACU at the HAL send.
bool can_tx_post(const CanFrame& f) noexcept;

}  // namespace ecu
