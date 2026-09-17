// SPDX-License-Identifier: proprietary
//
// GpsService -- the shared GPS solution, same single-writer pattern as
// VehicleService: GpsTask is the ONLY writer; TelemetryTask (radio snapshot)
// and anyone else take a snapshot().
//
// A GpsFix is 24 bytes of plain scalars and the writer is a single low-priority
// task, so a torn read would at worst mix two consecutive solutions 200 ms
// apart. The struct copy is done under a critical section anyway (it is
// cheap and makes the snapshot atomic) -- see gps_task.cpp.

#ifndef ECU_GPS_SERVICE_HPP_
#define ECU_GPS_SERVICE_HPP_

#include "app/gps_nmea.hpp"

#include <cstdint>

namespace ecu {

struct GpsState {
    GpsFix        fix{};
    std::uint32_t sentences   = 0;  // parsed since boot (liveness)
    std::uint32_t last_tick_ms = 0; // tick of the last parsed sentence (0 = never)
};

class GpsService {
public:
    static GpsService& instance() noexcept;

    // Writer (GpsTask only).
    void update(const GpsFix& fix, std::uint32_t sentences, std::uint32_t now_ms) noexcept;

    // Readers.
    GpsState snapshot() const noexcept;

private:
    GpsState state_{};
};

}  // namespace ecu

#endif  // ECU_GPS_SERVICE_HPP_
