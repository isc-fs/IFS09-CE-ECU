// SPDX-License-Identifier: proprietary
//
// Application-side entry into the stm32-can-bootloader -- the ECU's recovery
// path home. The BL (sector 0) reads RTC->BKP0R on every reset: if it holds
// BlBootReqMagic (0xB00710AD) the BL stays in its CAN listen loop awaiting a
// flash; otherwise it validates the app's firmware_info record and jumps to
// 0x08020000.
//
// request_reboot() is the one-way path from a running app back into the BL.
// It is triggered by a single CAN frame (id 0x002, payload 0xB007AD12) on the
// ACU/shared bus (FDCAN2) -- the same externally-accessible bus the pit tool
// and the flasher use. matches_trigger() is pure logic, kept inline so the host
// unit-test build exercises it without HAL/FreeRTOS.
//
// NOTE vs the AMS: the ECU owns no contactors, so request_reboot drives no
// relays. That does NOT make a reboot free while the car is driving, and the
// reason is on the other board. Resetting stops all ECU TX, including the
// 10 ms 0x100 heartbeat the AMS watches. In Car mode the AMS arms VcuStale at
// 200 ms and, when it fires, latches Error and opens the AIRs -- under whatever
// the inverter is drawing. Opening AIR+ under load is how contactors weld, and
// the AMS's ErrorLatch is sticky, so the car then boots back into Error needing
// a physical RST_BMS press the driver cannot reach.
//
// So a 4-byte frame anyone on the shared bus can send would take the car out
// mid-drive. reboot_allowed_in() below is the gate; the caller must consult it.

#pragma once

#include "app/can_frame.hpp"
#include "app/control.hpp"
#include "app/ecu_config.hpp"

#include <cstdint>
#include <cstring>

namespace ecu {

// Why we're rebooting into the BL -- stamped into BKP2R, survives the reset.
enum class JumpReason : std::uint32_t {
    ManualRequest = 1,   // a CAN 0x002 trigger from the pit tool
};

class Bootloader {
public:
    // Drain in-flight TX, stamp the reason into BKP2R, write BlBootReqMagic to
    // BKP0R, NVIC_SystemReset. Never returns. (Defined in bootloader.cpp, which
    // pulls the HAL -- not part of the host build.)
    [[noreturn]] static void request_reboot(
        JumpReason reason = JumpReason::ManualRequest) noexcept;

    // States in which honouring the reboot trigger is safe.
    //
    // The ECU's definition of "quiet" is NOT the AMS's. Theirs is "contactors
    // open"; ours is "not in the drive ladder" -- no torque commanded and no
    // R2D sequence under way. Same predicate Controller::enter_() uses to clear
    // the DV latch, deliberately: one definition of leaving the drive, not two.
    //
    // AmsError is included rather than grudgingly excepted. It is an inhibit
    // state with the car already stopped, and it is exactly when someone wants
    // to reflash. Refusing there would make a faulted car unrecoverable over
    // CAN, which is the failure this whole path exists to prevent.
    //
    // Every flashing workflow -- bench, pit tool, can-flasher, the VS Code
    // extension -- operates from these states, so the gate is invisible to them.
    //
    // THIS DEPENDS ON ControlTask KICKING THE IWDG, and nothing else does. A
    // wedged ControlTask would freeze the state mirror this reads, and if it
    // froze at Active the gate would refuse the reboot forever -- blocking
    // recovery exactly when it is needed. The watchdog is what makes that
    // impossible: a stalled ControlTask resets the board, which comes up in
    // WaitInvVdcConfig, where the gate opens. If the IWDG ever stops being
    // ControlTask's alone, re-check this.
    [[nodiscard]] static bool reboot_allowed_in(CtrlState s) noexcept {
        return s < CtrlState::R2dDelay || s == CtrlState::AmsError;
    }

    // True iff a frame is the boot-request trigger: the ACU bus, id 0x002,
    // dlc 4, payload 0xB007AD12. Pure -> host-testable.
    [[nodiscard]] static bool matches_trigger(const CanFrame& f) noexcept {
        if (f.bus != static_cast<std::uint8_t>(CanBus::Acu)) return false;
        if (f.id  != config::BlBootTriggerCanId)             return false;
        if (f.dlc != config::BlBootTriggerDlc)               return false;
        return std::memcmp(f.data, config::BlBootTriggerPayload,
                           config::BlBootTriggerDlc) == 0;
    }
};

}  // namespace ecu
