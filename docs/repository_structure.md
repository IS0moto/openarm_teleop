# Repository Structure

## Top-Level Layout

```text
openarm_wifi_bimanual_teleop/
  CMakeLists.txt
  README.md
  config/
  docs/
  include/openarm_wifi_teleop/
  script/
  src/
  tests/
  tools/
  urdf/
```

## Important Directories

| Path | Contents |
| --- | --- |
| `src/apps/` | Executable entry points: leaders, followers, VR IK leader. |
| `src/net/` | UDP sender/receiver and packet codec. |
| `src/core/` | Teleop state buffer and sequence/loss tracking. |
| `src/safety/` | Watchdog, rate limiting, safety manager. |
| `src/telemetry/` | OpenArm telemetry publisher. |
| `src/ik/` | Pinocchio IK implementation used by VR leader. |
| `src/controller/`, `src/openarm_port/` | OpenArm CAN/control integration. |
| `include/openarm_wifi_teleop/net/` | Binary UDP packet definitions. |
| `config/` | Runtime YAML configs for follower/leader gains and VR mapping/IK. |
| `urdf/` | Right/left OpenArm URDFs used by control and IK. |
| `script/` | Convenience launch, CAN, and network scripts. |
| `tests/` | Unit tests for codec, watchdog, rate limiter, and state buffer. |
| `tools/` | Telemetry receiver and mock telemetry publisher samples. |

## Runtime Components

### Physical Leader/Follower

```text
Physical leader arms
  -> wifi_bimanual_leader
  -> UDP TeleopPacket on ports 50000/50001
  -> wifi_bimanual_follower
  -> physical follower arms
```

### VR Controller Teleop

```text
Quest/OpenXR controllers
  -> openxr_relative_sender
  -> UDP VrRelativeTeleopPacketV2 on port 54002
  -> wifi_vr_bimanual_leader
  -> UDP TeleopPacket on ports 50000/50001
  -> wifi_bimanual_follower or openarm_udp_bridge/simulator
```

The VR leader also consumes telemetry on port `51000`.

### Telemetry

```text
wifi_bimanual_follower --publish-telemetry
  -> OpenArmTelemetryPacketV1 on port 51000
  -> wifi_vr_bimanual_leader, recorder, or telemetry tools
```
