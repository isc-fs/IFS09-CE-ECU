// SPDX-License-Identifier: proprietary

#include "app/error_latch.hpp"

#include "app/ecu_config.hpp"

#include "main.h"   // HAL + RTC

namespace ecu {

// BKP1R, addressed as a word offset from the BKP0R member (the registers are
// consecutive). Same direct-pointer idiom as the AMS error latch.
static inline volatile std::uint32_t* fault_reg() noexcept {
    return &((&RTC->BKP0R)[config::BkpFaultLatchReg]);
}

void FaultLatch::init() noexcept {
    HAL_PWR_EnableBkUpAccess();
}

void FaultLatch::set(FaultCode code) noexcept {
    HAL_PWR_EnableBkUpAccess();   // idempotent; cheap, and the hooks may run pre-init
    *fault_reg() = encode(code);
}

FaultCode FaultLatch::get() noexcept {
    return decode(*fault_reg());
}

void FaultLatch::clear() noexcept {
    HAL_PWR_EnableBkUpAccess();
    *fault_reg() = 0u;
}

extern "C" void ecu_fault_latch_set_c(std::uint8_t code) {
    FaultLatch::set(static_cast<FaultCode>(code));
}

}  // namespace ecu
