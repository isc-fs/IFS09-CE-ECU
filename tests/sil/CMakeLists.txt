cmake_minimum_required(VERSION 3.15)

# =============================================================================
# ECU SIL (Software-In-The-Loop) host build -- post-C++-rewrite.
#
# Compiles the pure control core (Core/Src/app/control.cpp -- HAL-free,
# RTOS-free) on the host with NO mocks and drives it through the start /
# ready-to-drive FSM + every FSAE plausibility cut (sil_control_tests.cpp).
#
# The C++ rewrite replaced the old C app (app_state.c / can.c / control.c / ...)
# that the previous SIL compiled, so this harness was rebuilt around the new
# ecu::Controller. The full RTOS-integration SIL (mocked tasks + CAN simulator)
# and the unit suite are tracked separately under #9.
#
# Run:  ctest --test-dir build-sil --output-on-failure
#   or: build-sil/tests/sil/ecu09_sil --test-all
# =============================================================================

set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_executable(ecu09_sil
    sil_control_tests.cpp
    ${REPO}/Core/Src/app/control.cpp
    ${REPO}/Core/Src/app/as_buzzer.cpp
    ${REPO}/Core/Src/app/cell_derate.cpp
    ${REPO}/Core/Src/app/discharge.cpp
    ${REPO}/Core/Src/app/motor_thermal.cpp
    ${REPO}/Core/Src/app/pack_thermal.cpp
    ${REPO}/Core/Src/app/pedal_cal.cpp
    ${REPO}/Core/Src/app/power_limit.cpp
    ${REPO}/Core/Src/app/pedal_cal_nvm.cpp
    ${REPO}/Core/Src/app/cal_session.cpp
    ${REPO}/Core/Src/app/inverter.cpp
    ${REPO}/Core/Src/app/udv_tx.cpp
    ${REPO}/Core/Src/app/radio_snapshot.cpp
    ${REPO}/Core/Src/app/vehicle_service.cpp
    ${REPO}/Core/Src/app/gps_nmea.cpp
    ${REPO}/Core/Src/app/gps_tx.cpp
)

target_include_directories(ecu09_sil PRIVATE ${REPO}/Core/Inc)

target_compile_options(ecu09_sil PRIVATE
    -Wall -Wextra -Wno-unused-parameter
    -fno-exceptions -fno-rtti          # mirror the firmware C++ flags
)
target_compile_definitions(ecu09_sil PRIVATE SIL_BUILD=1)
target_link_libraries(ecu09_sil m)

# Host tool: regenerate docs/dbc/ecu.dbc from the code-first CAN DSL. The
# dbcinator bot (.github/workflows/dbc-bot.yml, on every PR to dev/main) builds
# this target and force-pushes the regenerated DBC, so it must always build --
# and it's compiled by the SIL job too, so it must compile cleanly. C++ TU
# against the rewrite's C++ DSL (can_codecs.hpp / ECU_CAN_DESCRIPTORS ->
# ecu::ALL_MSGS of can_dsl::MsgDesc).
add_executable(ecu_dbc_dump ${REPO}/tools/dbc_dump.cpp)
target_include_directories(ecu_dbc_dump PRIVATE ${REPO}/Core/Inc)
target_compile_options(ecu_dbc_dump PRIVATE -Wall -Wextra)

enable_testing()
add_test(NAME SIL_Apps            COMMAND ecu09_sil --test-apps)
add_test(NAME SIL_Boot            COMMAND ecu09_sil --test-boot)
add_test(NAME SIL_FullCycle       COMMAND ecu09_sil --test-full-cycle)
add_test(NAME SIL_PrechargeNoAck  COMMAND ecu09_sil --test-precharge-no-ack)
add_test(NAME SIL_Plausibility    COMMAND ecu09_sil --test-plausibility)
add_test(NAME SIL_InvTsOffRecovery COMMAND ecu09_sil --test-inverter-ts-off)
add_test(NAME SIL_Gps             COMMAND ecu09_sil --test-gps)
add_test(NAME SIL_All             COMMAND ecu09_sil --test-all)
