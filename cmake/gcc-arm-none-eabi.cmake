# Cross-compilation toolchain for the STM32H733ZG (Cortex-M7, double-precision FPU).
# Used to build the ECU firmware for ARM:
#   cmake -S firmware -B build-fw -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake
# Compiler + MCU flags only; the linker script is selected by firmware/CMakeLists.txt.

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER_ID    GNU)
set(CMAKE_CXX_COMPILER_ID  GNU)

set(TOOLCHAIN_PREFIX       arm-none-eabi-)
set(CMAKE_C_COMPILER       ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER     ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER     ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY          ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE             ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_C   ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")

# Don't try to run target binaries during the compiler check.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb")

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${TARGET_FLAGS} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TARGET_FLAGS} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${TARGET_FLAGS} -x assembler-with-cpp -MMD -MP")

set(CMAKE_C_FLAGS_DEBUG     "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE   "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

# Newlib-nano; the app provides its own syscall stubs (Core/Src/syscalls.c, sysmem.c),
# so we do NOT add nosys.specs (that would duplicate _sbrk/_write/etc.).
set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS} --specs=nano.specs -Wl,--gc-sections -Wl,--print-memory-usage")
