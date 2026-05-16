# OpenArm Wi-Fi Bimanual Teleoperation

`openarm_wifi_bimanual_teleop` provides UDP-based teleoperation programs for OpenArm leader/follower systems and a VR IK leader for Meta Quest controller teleoperation.

The repository currently supports three practical workflows:

- **Two-PC Leader/Follower teleop**: physical OpenArm leader arms command physical follower arms over UDP.
- **Simulator/mock Leader/Follower teleop**: run the network and safety pipeline without CAN hardware, or bridge UDP commands into a ROS 2 simulator with the adjacent `openarm_udp_bridge` package.
- **VR controller/Follower teleop**: Quest/OpenXR controller deltas are converted into bimanual OpenArm joint targets by `wifi_vr_bimanual_leader`.

## Quick Links

- [Repository structure](docs/repository_structure.md)
- [Setup and installation](docs/setup_installation.md)
- [Two-PC Leader/Follower teleop](docs/leader_follower_two_pc.md)
- [Simulator and mock-mode Leader/Follower](docs/leader_follower_simulator.md)
- [VR controller teleop](docs/vr_controller_teleop.md)
- [CLI and configuration reference](docs/cli_reference.md)
- [Component tests and verification](docs/testing.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Safety model](docs/safety.md)
- [UDP protocols](docs/protocol.md)
- [Telemetry protocol](docs/telemetry_protocol.md)

## Main Binaries

| Binary | Purpose |
| --- | --- |
| `wifi_bimanual_leader` | Reads physical leader arms and sends right/left joint targets over UDP. |
| `wifi_bimanual_follower` | Receives UDP joint targets and drives physical follower arms. |
| `wifi_vr_bimanual_leader` | Receives Quest relative pose packets, solves IK with Pinocchio, and sends follower joint targets. |
| `wifi_single_leader` / `wifi_single_follower` | Single-arm variants for bring-up and debugging. |

The Quest sender lives in the sibling repository directory:

```bash
../quest2_openxr_logger/build/openxr_relative_sender
```

## Typical Build

```bash
cd openarm_wifi_bimanual_teleop
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

The current CMake file expects the OpenArm CAN install at:

```text
/home/d2301/openarm-ws/install/openarm_can
```

If your workspace differs, override `OPENARM_CAN_DIR` instead of hardcoding the path in `CMakeLists.txt`, for example with:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DOPENARM_CAN_DIR=/path/to/openarm_can
```

## Safety Notes

- Start with the robot supported and with an operator ready to stop the process.
- Initialize CAN in CAN FD mode before running real hardware.
- Start followers before leaders.
- For VR, follower telemetry must be enabled, otherwise the VR leader will hold commands.
- Use `--mock`, `--dry-run`, and low rate/scale settings for first tests.

For detailed safety behavior, see [Safety](docs/safety.md).
