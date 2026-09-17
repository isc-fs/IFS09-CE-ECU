cmake_minimum_required(VERSION 3.15)

# Descargar Unity si no está disponible
include(FetchContent)
FetchContent_Declare(Unity
    URL https://github.com/ThrowTheSwitch/Unity/archive/refs/tags/v2.5.4.tar.gz
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/_deps/unity-src
)

FetchContent_MakeAvailable(Unity)

# Fuentes del proyecto a compilar en host (sin RTOS, sin HAL)
# Solo módulos que NO dependen de hardware
set(PROJECT_SOURCES
    ../../Core/Src/can.c
    ../../Core/Src/control.c
    ../../Core/Src/telemetry.c
    ../../Core/Src/app_state.c
)

# Tests unitarios
set(TEST_SOURCES
    mocks.c
    test_can_parsing.c
    test_control_logic.c
    test_telemetry.c
    test_app_state.c
    test_error_handling.c
    unity_runner.c
)

# Crear executable
add_executable(ecu08_unit_tests
    ${PROJECT_SOURCES}
    ${TEST_SOURCES}
)

# Include directories
target_include_directories(ecu08_unit_tests PRIVATE
    ../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${Unity_SOURCE_DIR}/src
)

# Link libraries
target_link_libraries(ecu08_unit_tests PRIVATE
    unity
)

# Compilar con warnings y coverage
target_compile_options(ecu08_unit_tests PRIVATE
    -Wall -Wextra -pedantic
    -fprofile-arcs -ftest-coverage
)

target_link_options(ecu08_unit_tests PRIVATE
    -fprofile-arcs -ftest-coverage
)

# Add test target
enable_testing()
add_test(
    NAME UnitTests
    COMMAND ecu08_unit_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
