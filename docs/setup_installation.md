# Setup and Installation

This page describes the common setup for physical Leader/Follower teleop and VR teleop.

## System Requirements

- Ubuntu/Linux PC with CMake and a C++17 compiler.
- OpenArm CAN stack installed and available as a static library.
- `Eigen3`, `yaml-cpp`, Pinocchio, Orocos KDL, `kdl_parser`, and URDF libraries.
- PEAK or SocketCAN-compatible CAN adapters for real hardware.
- CAN FD support for OpenArm motor communication.
- Optional for simulator/ROS workflows: ROS 2 workspace containing `openarm_udp_bridge` and robot/simulator packages.
- Optional for VR: `quest2_openxr_logger` built with OpenXR support.

## Install Dependencies

Package names depend on your OpenArm workspace, but the following are the usual system dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  libeigen3-dev libyaml-cpp-dev \
  liborocos-kdl-dev liburdfdom-dev liburdfdom-headers-dev \
  iperf3 can-utils ros-humble-pinocchio \
  libcli11-dev
```

Install Pinocchio using the method used by your OpenArm workspace. The CMake project calls:

```cmake
find_package(pinocchio REQUIRED)
```

Install or build OpenArm CAN so that this path exists, or override `OPENARM_CAN_DIR` in CMake. The default is:

```text
/home/d2301/openarm-ws/install/openarm_can
```

You can override the path by environment variable or CMake option:

```bash
export OPENARM_CAN_DIR=/path/to/openarm_can
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release
```

or directly:

```bash
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release -D OPENARM_CAN_DIR=/path/to/openarm_can
```

## Build

```bash
cd openarm_wifi_bimanual_teleop
cmake -S . -B build -D CMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

The build produces:

```text
build/wifi_bimanual_leader
build/wifi_bimanual_follower
build/wifi_vr_bimanual_leader
build/wifi_single_leader
build/wifi_single_follower
```

## CAN FD Setup

OpenArm requires CAN FD. Run this after boot and before launching real-hardware programs:

```bash
sudo openarm-can-configure-socketcan can0 -fd
sudo openarm-can-configure-socketcan can1 -fd
```

If one PC has both leader and follower arms, you may also need:

```bash
sudo openarm-can-configure-socketcan can2 -fd
sudo openarm-can-configure-socketcan can3 -fd
```

Verify:

```bash
ip -details -statistics link show can0
candump can0 -t a
```

## URDF Files

The repo includes generated URDFs:

```text
urdf/openarm_right.urdf
urdf/openarm_left.urdf
```

Real hardware programs require URDF paths unless `--mock` is used. The VR IK leader defaults to these paths.

## Network Preparation

Find the IP/interface names:

```bash
ip addr show
```

Check connectivity:

```bash
ping <Follower_PC_IP>
```

Optional UDP load test:

```bash
iperf3 -s
./script/network_test_250hz.sh <Follower_PC_IP>
./script/network_test_500hz.sh <Follower_PC_IP>
```

Prefer a wired link or a dedicated 5 GHz/6 GHz Wi-Fi network for real hardware.

## Quest Sender Build

The Quest/OpenXR sender is in the sibling directory:

```bash
cd ../quest2_openxr_logger
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target openxr_relative_sender -j"$(nproc)"
```
