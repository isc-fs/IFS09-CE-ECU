// SPDX-License-Identifier: proprietary
//
// CanRxTask: drains can_rx_queue (filled by the FDCAN RX ISR) and dispatches
// each frame. Three jobs, in priority order: the bootloader trigger (the
// recovery path home), the pit-diag enable/disable command, and otherwise feed
// the VehicleService. It is the SOLE writer of VehicleService.

#include "app/app_tasks.h"

#include "app/app_globals.h"
#include "app/bootloader.hpp"
#include "app/can_tx.hpp"
#include "app/ecu_config.hpp"
#include "app/pit_diag.hpp"
#include "app/vehicle_service.hpp"

#include "can/can_codecs.hpp"

#include "cmsis_os2.h"
#include "main.h"

using namespace ecu;

extern "C" {
extern osMessageQueueId_t can_rx_queueHandle;
}

extern "C" void ecu_can_rx_task_run(void *argument) {
    (void)argument;

    auto&    vs = VehicleService::instance();
    CanFrame f;

    for (;;) {
        const osStatus_t st =
            osMessageQueueGet(can_rx_queueHandle, &f, NULL, config::CanRxWaitMs);
        // Liveness: bump on EVERY wake -- a frame OR the periodic timeout -- so a
        // healthy-but-idle RX task still advances 0x704's task_ran_mask bit on a
        // quiescent bus. The bit means "scheduled + running", not "saw traffic";
        // a genuine hang stops the wakes too, which is the stall it must catch.
        ++g_task_step[ECU_TASK_CAN_RX];
        if (st != osOK) {
            continue;   // timeout: no frame, just the liveness tick
        }

#if defined(ECU_DEBUG_INV_BRIDGE)
        // DEBUG: mirror every FDCAN1 (inverter) frame onto FDCAN2 so a pit tool on
        // the ACU bus can sniff the raw inverter traffic -- E2E CRC (byte0) + counter
        // (byte1) intact. Same ID: the 0x46x inverter IDs don't collide with the ACU
        // map. NEVER a flight default (doubles inverter traffic onto the shared bus).
        if (f.bus == static_cast<uint8_t>(CanBus::Inv)) {
            CanFrame fwd = f;
            fwd.bus = static_cast<uint8_t>(CanBus::Acu);
            can_tx_post(fwd);
        }
#endif

        // (1) bootloader trigger (0x002 / 0xB007AD12) -> reboot to BL. The
        // recovery path home; never returns.
        //
        // GATED ON BEING OUT OF THE DRIVE LADDER. The ECU drives no contactors,
        // but resetting stops the 10 ms 0x100 heartbeat, the AMS trips VcuStale
        // at 200 ms, and IT opens the AIRs -- under load. See
        // Bootloader::reboot_allowed_in for the full chain and for why this can
        // safely read a mirror written by another task.
        if (Bootloader::matches_trigger(f)) {
            const auto st = static_cast<CtrlState>(g_last_ctrl_state);
            if (Bootloader::reboot_allowed_in(st)) {
                Bootloader::request_reboot(JumpReason::ManualRequest);
            }
            // Refused. Count it rather than dropping it silently: an operator
            // whose flash did nothing needs to tell "the ECU said no" from
            // "the ECU is dead", and only one of those is fixed by dropping TS.
            ++g_boot_trigger_refused;
        }

        // (2) pit-diag enable/disable (0x7E0, magic 0xDEADBEEF) -> set the gate,
        // ack with 0x7E1.
        if (f.bus == static_cast<uint8_t>(CanBus::Acu) &&
            f.id == static_cast<uint32_t>(PitDiag_cmd_ID) && f.dlc >= 4u) {
            const uint32_t magic =
                (static_cast<uint32_t>(f.data[0]) << 24) |
                (static_cast<uint32_t>(f.data[1]) << 16) |
                (static_cast<uint32_t>(f.data[2]) << 8)  |
                 static_cast<uint32_t>(f.data[3]);
            const bool en = (magic == config::PitDiagEnableMagic);
            g_pit_diag_enabled = en ? 1u : 0u;
            can_tx_post(PitDiag::build_ack(en));
            continue;
        }

        // (3) calibration command (0x7E2) -> one-deep mailbox for ControlTask.
        // Decoded here but NOT acted on: the session needs the live pedal
        // readings and the live calibration, both of which ControlTask owns.
        if (f.bus == static_cast<uint8_t>(CanBus::Acu) &&
            f.id == static_cast<uint32_t>(PitCal_cmd_ID) && f.dlc >= 8u) {
            if (g_cal_cmd_pending == 0u) {
                g_cal_cmd   = f.data[0];
                g_cal_arg   = f.data[1];
                g_cal_guard = (static_cast<uint32_t>(f.data[4]) << 24) |
                              (static_cast<uint32_t>(f.data[5]) << 16) |
                              (static_cast<uint32_t>(f.data[6]) << 8)  |
                               static_cast<uint32_t>(f.data[7]);
                g_cal_cmd_pending = 1u;   // set LAST: the fields must be visible first
            }
            continue;
        }

        // (4) vehicle state: inverter (0x461/0x466) + AMS (0x020/0x12C/0x4A0).
        vs.update_from_frame(f);
    }
}
