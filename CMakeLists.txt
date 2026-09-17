cmake_minimum_required(VERSION 3.15)
project(ECU09_NSIL_Tests C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Opciones: compilar tests unitarios y/o SIL
option(BUILD_UNIT_TESTS "Build unit tests (Unity framework)" ON)
option(BUILD_SIL_TESTS "Build SIL tests (FreeRTOS simulation)" OFF)

# Top-level so `ctest --test-dir build-sil` discovers the add_test() calls
# registered in the tests/sil (+ tests/unit) subdirectories.
enable_testing()

# Detectar compilador para host
if(NOT CMAKE_C_COMPILER_WORKS)
    find_program(GCC_COMPILER gcc)
    if(GCC_COMPILER)
        set(CMAKE_C_COMPILER ${GCC_COMPILER})
    endif()
endif()

# ===== UNIT TESTS (tests/unit/) =====
if(BUILD_UNIT_TESTS)
    message(STATUS "Building UNIT TESTS with ${CMAKE_C_COMPILER}")
    add_subdirectory(tests/unit)
else()
    message(STATUS "Unit tests DISABLED. To enable: -DBUILD_UNIT_TESTS=ON")
endif()

# ===== SIL TESTS (tests/sil/) =====
if(BUILD_SIL_TESTS)
    message(STATUS "Building SIL TESTS with FreeRTOS simulation")
    add_subdirectory(tests/sil)
else()
    message(STATUS "SIL tests DISABLED. To enable: -DBUILD_SIL_TESTS=ON")
endif()

# ===== BUILD INFORMATION =====
message(STATUS "")
message(STATUS "=== ECU09 NSIL Test Build Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Build directory: ${CMAKE_BINARY_DIR}")
message(STATUS "")
message(STATUS "Unit Tests: ${BUILD_UNIT_TESTS}")
message(STATUS "SIL Tests: ${BUILD_SIL_TESTS}")
message(STATUS "==========================================")
message(STATUS "")
