# ============================================================================
# ECU09 firmware — arm-none-eabi cross-compile for STM32H733ZG.
#
#   cmake -S firmware -B build-fw -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake
#   cmake --build build-fw
#
# Produces build-fw/ECU09.elf. CubeMX-generated C (main/freertos/peripherals)
# plus the hand-written C++17 application in Core/Src/app/. The host SIL/unit
# tests live at the repo-root CMakeLists and are unaffected.
#
# NOTE: depends on a lean CubeMX regen of ECU.ioc (FDCAN1+2 only, the 6 tasks +
# 2 queues, no SPI/I2C/FDCAN3, the startup_*.s). Globs Core/Src so it tracks the
# regen output automatically -- re-run cmake configure after a regen adds files.
# ============================================================================
cmake_minimum_required(VERSION 3.15)

project(ECU09 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Repo root (this file is in firmware/).
set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(LINKER_SCRIPT ${REPO}/STM32H733ZGTX_FLASH.ld)

# --- Firmware identity baked into the bootloader fwinfo record -------------
# (Core/Src/app/firmware_info.cpp). Git short hash split into 4 bytes; build epoch.
find_package(Git QUIET)
set(ECU_GIT_SHORT "00000000")
if(Git_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} -C ${REPO} rev-parse --short=8 HEAD
    OUTPUT_VARIABLE ECU_GIT_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
endif()
string(SUBSTRING "${ECU_GIT_SHORT}00000000" 0 8 ECU_GIT_SHORT)
string(SUBSTRING "${ECU_GIT_SHORT}" 0 2 H0)
string(SUBSTRING "${ECU_GIT_SHORT}" 2 2 H1)
string(SUBSTRING "${ECU_GIT_SHORT}" 4 2 H2)
string(SUBSTRING "${ECU_GIT_SHORT}" 6 2 H3)
string(TIMESTAMP ECU_BUILD_TS "%s" UTC)

add_compile_definitions(
  USE_HAL_DRIVER STM32H733xx USE_PWR_LDO_SUPPLY
  ECU_FW_VERSION_MAJOR=0 ECU_FW_VERSION_MINOR=1 ECU_FW_VERSION_PATCH=0
  ECU_GIT_HASH_0=0x${H0} ECU_GIT_HASH_1=0x${H1}
  ECU_GIT_HASH_2=0x${H2} ECU_GIT_HASH_3=0x${H3}
  ECU_BUILD_TIMESTAMP=${ECU_BUILD_TS}ULL
)

# Bench image only -- wipes a stale fault latch on boot. NEVER a flight default.
option(ECU_HIL_CLEAR_ERROR_LATCH "Clear the RTC fault latch at boot (HIL only)" OFF)
if(ECU_HIL_CLEAR_ERROR_LATCH)
  add_compile_definitions(ECU_HIL_CLEAR_ERROR_LATCH)
endif()

# The bench/bring-up stubs (brake, start button, no-AMS, no-inverter) and the torque
# cap are now SOURCE CONSTANTS in ecu_config.hpp (config::StubBrakeRaw / StubStart /
# StubNoAms / StubNoInverter / TorqueCap), consumed via `if constexpr` / `!= 0` so a
# false toggle folds away at compile time -- not build flags. See docs/commissioning.md.

# Debug image only -- mirror every FDCAN1 (inverter) frame onto FDCAN2 so a pit tool on
# the ACU bus can sniff the raw inverter traffic (E2E CRC/counter intact). NEVER a flight default.
option(ECU_DEBUG_INV_BRIDGE "Mirror FDCAN1 inverter frames onto FDCAN2 (debug only)" OFF)
if(ECU_DEBUG_INV_BRIDGE)
  add_compile_definitions(ECU_DEBUG_INV_BRIDGE)
endif()

# C++ app flags: no exceptions / RTTI / threadsafe-static guards (mirrors AMS).
add_compile_options(
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
)

include_directories(
  ${REPO}/Core/Inc
  ${REPO}/Drivers/STM32H7xx_HAL_Driver/Inc
  ${REPO}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
  ${REPO}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
  ${REPO}/Drivers/CMSIS/Include
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/include
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
  ${REPO}/Drivers/CMSIS/RTOS2/Include
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

# CubeMX-generated C (main, freertos, peripherals) + the hand-written C++ app.
file(GLOB CUBEMX_C   ${REPO}/Core/Src/*.c)
file(GLOB APP_CPP    ${REPO}/Core/Src/app/*.cpp)
file(GLOB STARTUP_S  ${REPO}/Core/Startup/*.s)

# --- STM32 HAL (compile all; unused modules are #ifdef'd out by hal_conf) ---
file(GLOB HAL_SRC ${REPO}/Drivers/STM32H7xx_HAL_Driver/Src/*.c)
list(FILTER HAL_SRC EXCLUDE REGEX ".*_template\\.c$")

# --- FreeRTOS (CMSIS-RTOS v2, ARM_CM4F port, heap_4) -----------------------
set(RTOS_SRC
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/list.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/queue.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/timers.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
  ${REPO}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

add_executable(ECU09
  ${CUBEMX_C} ${APP_CPP} ${HAL_SRC} ${RTOS_SRC} ${STARTUP_S}
)

target_link_options(ECU09 PRIVATE
  -T${LINKER_SCRIPT}
  -Wl,-Map=ECU09.map,--cref
  -Wl,--gc-sections
  -Wl,--print-memory-usage
  -lc -lm
)

# Print size after build.
add_custom_command(TARGET ECU09 POST_BUILD
  COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:ECU09>)
