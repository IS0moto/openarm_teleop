# OpenArm VR Bimanual Teleoperation System

This document describes the bimanual teleoperation system using Quest 2 controllers and the OpenArm robot (real or simulated).

## System Architecture

The system consists of three main components communicating via UDP and ROS 2:

1.  **Quest App (`openxr_relative_sender`)**: Runs on the PC (via WiVRn) or Quest. It captures controller poses, calculates relative displacement (Delta) from the moment the Grip is pressed, and sends UDP packets to the Leader PC.
2.  **Leader App (`wifi_vr_bimanual_leader`)**: Receives Quest data and robot telemetry. It runs Inverse Kinematics (Pinocchio) to calculate target joint angles based on the current robot position + Quest Delta.
3.  **UDP-to-ROS 2 Bridge (`udp_to_ros2_bridge`)**: A Python node that bridges UDP teleop packets to ROS 2 topics (`/follower/.../commands`) and sends back robot state as UDP telemetry.

## Component Setup

### 1. Robot / Simulator
Launch the robot or simulation environment:
```bash
# Real hardware or specialized follower config
ros2 launch openarm_teleop_ros2 follower.launch.py
```

### 2. UDP-ROS 2 Bridge
Connects the Wi-Fi teleop protocol to the ROS 2 control system:
```bash
cd ~/openarm-ws
colcon build --packages-select openarm_udp_bridge
source install/setup.bash
ros2 run openarm_udp_bridge udp_to_ros2_bridge
```

### 3. VR Leader PC
Handles IK calculation and coordination:
```bash
cd ~/openarm_isolate_teleop/openarm_wifi_bimanual_teleop
./build/wifi_vr_bimanual_leader --follower-ip 127.0.0.1
```

### 4. Quest Sender
Captures tracking data:
```bash
cd ~/openarm_isolate_teleop/quest2_openxr_logger
./build/openxr_relative_sender
```

## Key Features & Safety

- **Relative Coordinate Control**: Prevents "jumps" when starting teleop. The robot moves relative to its position at the moment of gripping.
- **Robust IK Solver**: Uses Pinocchio with Damped Least Squares and `CompleteOrthogonalDecomposition` to handle singularities and prevent NaN failures.
- **Watchdog & Safety**: The system automatically disables robot movement if UDP packets are lost or if IK produces invalid (NaN) results.

## Troubleshooting

- **Robot not moving**:
    - Verify `ros2 topic list` shows `/follower/joint_states`.
    - Check if `wifi_vr_bimanual_leader` shows `VR: OK` and `TEL: OK`.
    - Ensure you are holding the **Grip** button to enable movement.
- **Movement is jerky**:
    - Check network latency (ideally use a dedicated 5GHz Wi-Fi or wired connection).
    - Verify if IK solver is rejecting "jumps" in the logs.
