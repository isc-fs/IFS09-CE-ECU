// SPDX-License-Identifier: proprietary

#include "app/bootloader.hpp"

#include "app/ecu_config.hpp"

#include "cmsis_os2.h"
#include "main.h"

namespace ecu {

void Bootloader::request_reboot(JumpReason reason) noexcept {
    // 1) Let any in-flight TX (the ACK to whoever sent the trigger, a last
    // heartbeat) clear the FDCAN controller before we reset. 10 ms is plenty
    // at 500 kbps -- a full 8-byte frame is < 200 us on the wire.
    osDelay(10);

    // 2) Hand the bootloader its magic in the backup-domain words it reads on
    // every reset. Stamp the reason FIRST and the boot-request magic LAST: if a
    // reset interrupts us mid-sequence we'd rather land in app-mode with a
    // stale reason than in BL-mode with no reason at all.
    HAL_PWR_EnableBkUpAccess();
    (&RTC->BKP0R)[config::BkpJumpReasonReg] = static_cast<std::uint32_t>(reason);
    (&RTC->BKP0R)[config::BkpBootReqReg]    = config::BlBootReqMagic;

    // 3) HW reset. The BL reset handler reads BKP0R, sees the magic, clears it
    // (one-shot) and stays in BL mode awaiting a flash. Never returns.
    NVIC_SystemReset();

    // Unreachable -- silences [[noreturn]] warnings on toolchains that don't
    // recognise NVIC_SystemReset as no-return.
    for (;;) {}
}

}  // namespace ecu
