// SPDX-License-Identifier: proprietary
//
// GpsTask -- MTK3339 GPS on USART10 (PG11 RX / PG12 TX, 9600 8N1).
//
// Transport: the USART10 RX interrupt pushes one byte at a time into a
// lock-free SPSC ring; this task drains the ring, feeds the pure NMEA parser
// (gps_nmea.cpp) and publishes:
//   - GpsService  -> read by TelemetryTask for the radio snapshot
//   - 0x508/0x509 -> ACU bus at GpsTxPeriodMs (uDV localisation + pit tool)
//
// Deliberately NOT the bench driver's blocking HAL_UART_Receive: that would
// park an RTOS task on the UART. Nothing here blocks -- the ISR is a few
// instructions and the task sleeps on osDelay between drains.
//
// The ISR touches NO FreeRTOS API (a plain SPSC ring with a single producer and
// a single consumer), so the USART10 priority-5 NVIC setting CubeMX generates
// is safe regardless of configMAX_SYSCALL_INTERRUPT_PRIORITY.

#include "app/app_tasks.h"

#include "app/can_tx.hpp"
#include "app/ecu_config.hpp"
#include "app/gps_nmea.hpp"
#include "app/gps_service.hpp"
#include "app/gps_tx.hpp"

#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"      // huart10

using namespace ecu;

// ---- RX ring (ISR producer / task consumer) --------------------------------
namespace {

constexpr unsigned kRingSize = config::GpsRxRingSize;   // power of two
static_assert((kRingSize & (kRingSize - 1u)) == 0u, "ring size must be a power of two");

volatile std::uint16_t s_head = 0;   // written by the ISR only
volatile std::uint16_t s_tail = 0;   // written by the task only
std::uint8_t           s_ring[kRingSize];
std::uint8_t           s_rx_byte = 0;

// Bytes dropped because the task did not drain in time. Non-zero means the GPS
// task is starved -- surfaced nowhere yet, but invaluable under a debugger.
volatile std::uint32_t s_overruns = 0;

inline bool ring_pop(std::uint8_t& out) noexcept {
    const std::uint16_t tail = s_tail;
    if (tail == s_head) return false;               // empty
    out = s_ring[tail];
    s_tail = static_cast<std::uint16_t>((tail + 1u) & (kRingSize - 1u));
    return true;
}

}  // namespace

// Single RX-complete callback for the whole firmware (nothing else uses a UART
// in interrupt mode -- NRF24_DiagnoseSerial is dormant and polls). Re-arms the
// single-byte receive every time; at 9600 baud that is ~960 interrupts/s.
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance != USART10) return;

    const std::uint16_t head = s_head;
    const std::uint16_t next = static_cast<std::uint16_t>((head + 1u) & (kRingSize - 1u));
    if (next != s_tail) {                 // drop the byte if the task fell behind
        s_ring[head] = s_rx_byte;
        s_head = next;
    } else {
        ++s_overruns;
    }

    (void)HAL_UART_Receive_IT(huart, &s_rx_byte, 1u);
}

namespace {

// Send a PMTK command (adds the "*CS\r\n" checksum tail). Blocking, but this
// runs ONCE at task start before the RX stream matters, and the payloads are
// ~20 bytes at 9600 baud (~20 ms) -- never in a control path.
void pmtk_send(const char* body) noexcept {
    char msg[64];
    unsigned n = 0;
    msg[n++] = '$';
    std::uint8_t cs = 0;
    for (const char* p = body; *p && n < (sizeof(msg) - 6u); ++p) {
        msg[n++] = *p;
        cs = static_cast<std::uint8_t>(cs ^ static_cast<std::uint8_t>(*p));
    }
    const char hex[] = "0123456789ABCDEF";
    msg[n++] = '*';
    msg[n++] = hex[(cs >> 4) & 0x0Fu];
    msg[n++] = hex[cs & 0x0Fu];
    msg[n++] = '\r';
    msg[n++] = '\n';
    (void)HAL_UART_Transmit(&huart10, reinterpret_cast<std::uint8_t*>(msg),
                            static_cast<std::uint16_t>(n), 100u);
}

}  // namespace

extern "C" void ecu_gps_task_run(void* argument) {
    (void)argument;

    GpsNmea parser;

    // Configure the module: emit ONLY RMC+GGA (everything else is UART noise we
    // would parse and discard) and raise the fix rate from the 1 Hz default.
    // If the module ignores these it simply stays at its defaults -- the parser
    // does not care. PMTK314 field order: GLL,RMC,VTG,GGA,GSA,GSV,...
    pmtk_send("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
    pmtk_send(config::GpsPmtkUpdateRate);

    // Arm the interrupt-driven receive; every completion re-arms itself.
    (void)HAL_UART_Receive_IT(&huart10, &s_rx_byte, 1u);

    std::uint32_t tick    = osKernelGetTickCount();
    std::uint32_t last_tx = tick;

    for (;;) {
        // Drain everything the ISR has buffered since the last pass.
        std::uint8_t b;
        while (ring_pop(b)) {
            if (parser.feed(static_cast<char>(b))) {
                GpsService::instance().update(parser.fix(), parser.sentences(),
                                              osKernelGetTickCount());
            }
        }

        // Publish at a fixed cadence regardless of sentence arrival, so the uDV
        // sees a steady stream (and a stale/absent GPS still shows has_fix=0
        // with a frozen nmea_count rather than silence).
        const std::uint32_t now = osKernelGetTickCount();
        if (static_cast<std::uint32_t>(now - last_tx) >= config::GpsTxPeriodMs) {
            last_tx = now;
            const GpsState g = GpsService::instance().snapshot();
            can_tx_post(GpsTx::build_position(g.fix));
            can_tx_post(GpsTx::build_status(g.fix, g.sentences));
        }

        tick += config::GpsPollPeriodMs;
        osDelayUntil(tick);
    }
}
