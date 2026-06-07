cmake_minimum_required(VERSION 3.10)
project(openarm_wifi_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------
# Dependencies
# -----------------------------
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
find_package(pinocchio REQUIRED)

# -----------------------------
# External OpenArm CAN Paths
# -----------------------------
if(NOT DEFINED OPENARM_CAN_DIR)
    if(DEFINED ENV{OPENARM_CAN_DIR})
        set(OPENARM_CAN_DIR "$ENV{OPENARM_CAN_DIR}" CACHE PATH "Path to installed OpenArm CAN prefix")
    else()
        set(OPENARM_CAN_DIR "/home/d2301/openarm-ws/install/openarm_can" CACHE PATH "Path to installed OpenArm CAN prefix")
    endif()
else()
    set(OPENARM_CAN_DIR "${OPENARM_CAN_DIR}" CACHE PATH "Path to installed OpenArm CAN prefix")
endif()
set(OPENARM_CAN_INC "${OPENARM_CAN_DIR}/include")
set(OPENARM_CAN_LIB "${OPENARM_CAN_DIR}/lib/libopenarm_can.a")

# -----------------------------
# Includes
# -----------------------------
include_directories(
    include
    src
    ${EIGEN3_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
    ${kdl_parser_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${OPENARM_CAN_INC}
)

# -----------------------------
# OpenArm Core Library
# -----------------------------
add_library(openarm_teleop_lib STATIC
    src/controller/control.cpp
    src/controller/dynamics.cpp
    src/openarm_port/openarm_init.cpp
    src/openarm_port/joint_mapper.cpp
)
target_link_libraries(openarm_teleop_lib PUBLIC
    orocos-kdl
    kdl_parser::kdl_parser
    urdf::urdf
    ${Eigen3_LIBRARIES}
    ${OPENARM_CAN_LIB}
)

# -----------------------------
# Wi-Fi Teleop Library
# -----------------------------
add_library(openarm_wifi_teleop_lib STATIC
    src/net/udp_sender.cpp
    src/net/udp_receiver.cpp
    src/net/packet_codec.cpp
    src/utils/network.cpp
    src/utils/logging.cpp
    src/telemetry/openarm_telemetry_publisher.cpp
    src/ik/openarm_pinocchio_ik.cpp
    src/control/runtime_control_server.cpp
    src/safety/watchdog.cpp
    src/safety/rate_limiter.cpp
    src/safety/safety_manager.cpp
    src/core/teleop_state_buffer.cpp
)

target_link_libraries(openarm_wifi_teleop_lib PUBLIC 
    openarm_teleop_lib
    yaml-cpp::yaml-cpp
    pinocchio::pinocchio
    pthread
)

# -----------------------------
# Applications
# -----------------------------

macro(add_wifi_teleop_app name src)
    add_executable(${name} ${src})
    target_link_libraries(${name} PRIVATE 
        openarm_wifi_teleop_lib
        pthread
    )
endmacro()

add_wifi_teleop_app(wifi_bimanual_leader src/apps/wifi_bimanual_leader.cpp)
add_wifi_teleop_app(wifi_bimanual_follower src/apps/wifi_bimanual_follower.cpp)
add_wifi_teleop_app(wifi_vr_bimanual_leader src/apps/wifi_vr_bimanual_leader.cpp)
add_wifi_teleop_app(wifi_single_leader src/apps/wifi_single_leader.cpp)
add_wifi_teleop_app(wifi_single_follower src/apps/wifi_single_follower.cpp)

add_executable(telemetry_fanout tools/telemetry_fanout.cpp)
add_executable(udp_fanout tools/udp_fanout.cpp)
