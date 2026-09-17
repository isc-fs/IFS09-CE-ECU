// SPDX-License-Identifier: proprietary
//
// See gps_service.hpp. Writer = GpsTask, readers = TelemetryTask (and any
// future consumer). Both are TASKS (never an ISR), so a FreeRTOS critical
// section is the right and cheapest way to make the struct copy atomic --
// without it a preemption mid-copy could hand out a position from one
// solution and a speed from the next.

#include "app/gps_service.hpp"

#include "FreeRTOS.h"
#include "task.h"

namespace ecu {

GpsService& GpsService::instance() noexcept {
    static GpsService Instance;
    return Instance;
}

void GpsService::update(const GpsFix& fix, std::uint32_t sentences,
                        std::uint32_t now_ms) noexcept {
    taskENTER_CRITICAL();
    state_.fix          = fix;
    state_.sentences    = sentences;
    state_.last_tick_ms = now_ms;
    taskEXIT_CRITICAL();
}

GpsState GpsService::snapshot() const noexcept {
    taskENTER_CRITICAL();
    const GpsState copy = state_;
    taskEXIT_CRITICAL();
    return copy;
}

}  // namespace ecu
