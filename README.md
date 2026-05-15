# OpenArm Wi-Fi Bimanual Teleoperation

This repository provides a distributed teleoperation system for OpenArm robots over Wi-Fi. It supports two main operation modes: **Standard Leader Mode** (using a physical OpenArm as a master) and **VR Teleop Mode** (using Meta Quest 2 controllers).

## Teleoperation Modes

### 1. Standard Leader Mode (Arm-to-Arm)
In this mode, a physical OpenArm (Leader) is used to control another OpenArm (Follower).
- **Control Logic**: Direct joint-to-joint mapping (or gravity-compensated passive lead-through).
- **Binary**: `wifi_bimanual_leader`
- **Setup**: Requires a Leader arm connected to the network.

### 2. VR Teleop Mode (Quest 2)
In this mode, Meta Quest 2 controllers are used to move the robot's end-effectors in Cartesian space.
- **Control Logic**: Relative Cartesian displacement (Delta) converted to joint angles via **Pinocchio IK**.
- **Binary**: `wifi_vr_bimanual_leader`
- **Setup**: Requires a PC running the Quest sender and a bridge to ROS 2.

---

## Usage Instructions

### Prerequisites
1.  **Robot/Sim Side**: Launch the follower robot controller.
    ```bash
    ros2 launch openarm_teleop_ros2 follower.launch.py
    ```
2.  **UDP Bridge**: Launch the bridge between UDP and ROS 2.
    ```bash
    ros2 run openarm_udp_bridge udp_to_ros2_bridge
    ```

### Mode A: Using physical Leader Arm
Run the standard leader application on the Leader PC:
```bash
./build/wifi_bimanual_leader --follower-ip <FOLLOWER_IP>
```

### Mode B: Using Meta Quest 2
1.  **Start the VR Leader App**: This handles IK calculations.
    ```bash
    ./build/wifi_vr_bimanual_leader --follower-ip 127.0.0.1
    ```
2.  **Start the Quest Sender**: Run this on the machine connected to Quest (via WiVRn or Link).
    ```bash
    cd quest2_openxr_logger
    ./build/openxr_relative_sender
    ```
3.  **Operation**: Hold the **Grip** button to start moving. The robot moves relative to its current position at the moment you press the grip (Clutch/Anchor logic).

---

## Build Instructions

### Dependencies
- **Pinocchio**: Required for VR Mode (IK).
- **Eigen3**: Linear algebra.

### Build Steps
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Documentation
For more detailed information on the VR system architecture and troubleshooting, see:
- [VR System Setup Guide](docs/teleop_system_setup.md)
- [UDP Bridge README](../openarm_udp_bridge/README.md)
