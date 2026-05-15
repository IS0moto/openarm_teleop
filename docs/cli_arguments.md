# CLI Arguments and Configuration Reference

This document provides a detailed reference for the command-line arguments and configuration files used in the OpenArm teleoperation system.

## 1. wifi_vr_bimanual_leader (VR IK Leader)
Handles Inverse Kinematics for Quest 2 controller inputs.

| Argument | Default | Description |
|----------|---------|-------------|
| `--follower-ip` | `172.30.21.146` | IP address of the robot/bridge PC. |
| `--vr-port` | `54000` | UDP port to listen for Quest 2 packets. |
| `--telemetry-port` | `51000` | UDP port to listen for robot telemetry. |
| `--rate-hz` | `100.0` | Target rate for the IK control loop. |
| `--right-urdf` | `urdf/openarm_right.urdf` | Path to the right arm URDF for IK. |
| `--left-urdf` | `urdf/openarm_left.urdf` | Path to the left arm URDF for IK. |

## 2. wifi_bimanual_leader (Physical Leader)
Handles direct joint-to-joint teleoperation using physical arms.

| Argument | Default | Description |
|----------|---------|-------------|
| `--follower-ip` | - | **Required**. IP of the follower robot. |
| `--right-can` | `can0` | CAN interface for the right leader arm. |
| `--left-can` | `can1` | CAN interface for the left leader arm. |
| `--interface` | - | Network interface name (e.g., `eth0`) to auto-detect bind IP. |
| `--bind-ip` | `0.0.0.0` | Explicit IP to bind UDP senders to. |
| `--mock` | `false` | Enable mock mode (no CAN hardware required). |
| `--enable` | `false` | Enable motors on startup. |
| `--publish-telemetry`| `false` | Re-publish leader states as UDP telemetry. |

## 3. openxr_relative_sender (Quest Sender)
Captures XR tracking data and sends relative displacement.

| Argument | Default | Description |
|----------|---------|-------------|
| `--config` | `configs/vr_relative_mapping.yaml` | Path to the mapping and scale config. |
| `--print` | `false` | Enable verbose logging of tracking data. |

### Configuration File (`vr_relative_mapping.yaml`)
- `openarm_teleop_ip`: Target Leader PC IP.
- `scale_xyz`: Multiplier for movement in each axis.
- `axis_matrix`: 3x3 rotation matrix to align Quest space with Robot space.
- `max_delta_m`: Safety limit for maximum displacement per clutch (m).

## 4. udp_to_ros2_bridge (ROS 2 Bridge)
Bridge parameters are set via ROS 2 parameter system.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `leader_ip` | `127.0.0.1` | Where to send robot telemetry (Leader PC IP). |
| `right_port` | `50000` | UDP port for inbound Right arm commands. |
| `left_port` | `50001` | UDP port for inbound Left arm commands. |
| `telemetry_port` | `51000` | UDP port for outbound Telemetry. |

**Example usage**:
```bash
ros2 run openarm_udp_bridge udp_to_ros2_bridge --ros-args -p leader_ip:=192.168.1.10
```
