cmake_minimum_required(VERSION 3.14)
project(example_soem LANGUAGES C CXX)

if(MSVC AND MSVC_VERSION LESS 1920)
    # Visual Studio 2017 has no /std:c11 switch, but accepts the C features
    # used by this project as compiler extensions.
    set(CMAKE_C_STANDARD 90)
else()
    set(CMAKE_C_STANDARD 11)
endif()
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(CTest)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(SOEM_BUILD_SAMPLES OFF CACHE BOOL "Build bundled SOEM samples")
if(NOT TARGET soem)
    add_subdirectory(external/SOEM)
endif()

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Network)

add_library(example_soem_core STATIC
    src/demo_timing.c
    src/ethercat_layer.c
    src/motor_layer.c
    src/motor_models.c
)

target_include_directories(example_soem_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external/SOEM/include
)

target_link_libraries(example_soem_core PUBLIC soem)
if(NOT MSVC)
    target_link_libraries(example_soem_core PUBLIC m)
endif()

if(MSVC)
    target_compile_definitions(example_soem_core PUBLIC _CRT_SECURE_NO_WARNINGS)
endif()

add_executable(example_soem_demo examples/main.c)
target_link_libraries(example_soem_demo PRIVATE example_soem_core)

add_executable(tcp_receive_server_8motors examples/tcp_receive_server_8motors.c)
target_link_libraries(tcp_receive_server_8motors PRIVATE example_soem_core)

add_executable(udp_receive_server_8motors examples/udp_receive_server_8motors.c)
target_link_libraries(udp_receive_server_8motors PRIVATE example_soem_core)

add_executable(tcp_can_test_tool tools/tcp_can_test_tool.cpp)
target_link_libraries(tcp_can_test_tool PRIVATE example_soem_core)

add_executable(udp_one_motor_bridge_test tools/udp_one_motor_bridge_test.cpp)
target_link_libraries(udp_one_motor_bridge_test PRIVATE example_soem_core)

if(WIN32)
    target_link_libraries(tcp_receive_server_8motors PRIVATE ws2_32)
    target_link_libraries(udp_receive_server_8motors PRIVATE ws2_32)
    target_link_libraries(tcp_can_test_tool PRIVATE ws2_32)
    target_link_libraries(udp_one_motor_bridge_test PRIVATE ws2_32)
endif()

add_executable(qt_motor_control_ui tools/qt_motor_control_ui.cpp)
set_target_properties(qt_motor_control_ui PROPERTIES AUTOMOC ON)
target_link_libraries(qt_motor_control_ui PRIVATE example_soem_core Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Network)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(socketcan_8motor_simulator tools/socketcan_8motor_simulator.cpp)
    target_link_libraries(socketcan_8motor_simulator PRIVATE example_soem_core)
endif()

if(BUILD_TESTING)
    add_executable(example_soem_tests tests/test_core.c)
    target_link_libraries(example_soem_tests PRIVATE example_soem_core)
    add_test(NAME example_soem_tests COMMAND example_soem_tests)
endif()
