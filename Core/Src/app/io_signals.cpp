// SPDX-License-Identifier: proprietary

#include "app/io_signals.hpp"

#include "app/ecu_config.hpp"  // config::StubBrakeRaw (brake stub value/enable)

#include "adc.h"     // hadc3
#include "main.h"    // HAL + the CubeMX GPIO label defines (START_Pin, ...)

namespace ecu {
namespace {

// One single-shot ADC3 conversion on `channel`. Safe-fails to 0 on any HAL
// error -- a bad read must look like "pedal released", never a stale value
// (FSAE: an APPS fault drives torque to 0).
std::uint16_t read_adc3(std::uint32_t channel) noexcept {
    ADC_ChannelConfTypeDef cfg = {};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;
    cfg.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc3, &cfg) != HAL_OK) return 0;
    if (HAL_ADC_Start(&hadc3) != HAL_OK)               return 0;
    std::uint16_t v = 0;
    if (HAL_ADC_PollForConversion(&hadc3, 2u) == HAL_OK) {
        v = static_cast<std::uint16_t>(HAL_ADC_GetValue(&hadc3));
    }
    HAL_ADC_Stop(&hadc3);
    return v;
}

}  // namespace

void IoSignals::read(IoInputs& out) noexcept {
    // ADC3 channels: brake IN3 (PF7), APPS1 IN7 (PF8, shared-analog), APPS2 IN2 (PF9).
    // Brake stub is a CONFIG VALUE, not a build flag: config::StubBrakeRaw != 0
    // injects it as the brake reading (enable AND value both in ecu_config.hpp);
    // 0 reads the real ADC (flight). Because StubBrakeRaw is constexpr this folds
    // at compile time — a StubBrakeRaw==0 build carries only the ADC read. Set it
    // ABOVE BrakeDvHardRaw (2500) to arm the DV R2D (bench: 2700). NEVER
    // nonzero for flight.
    if constexpr (config::StubBrakeRaw != 0u) {
        out.brake_raw = config::StubBrakeRaw;
    } else {
        out.brake_raw = read_adc3(ADC_CHANNEL_3);
    }
    out.apps1_raw = read_adc3(ADC_CHANNEL_7);
    out.apps2_raw = read_adc3(ADC_CHANNEL_2);

    // Bench stub (config::StubStart, ecu_config.hpp — folds away when false): the
    // start button may not be wired, so assume it's pressed (PB5 isn't read).
    // config value, NOT a build flag -- never true for flight. ⚠ keep false for a
    // DV/uDV R2D test: manual start preempts the dv_r2d_req path (control.cpp).
    bool pressed;
    if constexpr (config::StubStart) {
        pressed = true;
    } else {
        pressed = (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == GPIO_PIN_SET);
    }
    out.start_button = debounce_start(pressed);
}

}  // namespace ecu
