# VR Controller to Follower Teleop

This workflow uses Meta Quest/OpenXR controller poses to command OpenArm end-effector targets through IK.

## Components

- `openxr_relative_sender` from `quest2_openxr_logger`
  - Sends controller relative deltas.
  - Defaults to `position=grip pose`, `rotation=aim pose`.
- `wifi_vr_bimanual_leader`
  - Receives Quest packets on `--vr-port` (`54002` by default).
  - Receives follower telemetry on `--telemetry-port` (`51000` by default).
  - Solves IK and sends joint targets to the follower.
- Follower side
  - Either `wifi_bimanual_follower --publish-telemetry` for hardware, or `openarm_udp_bridge` for ROS 2/simulator workflows.

## Default Pose Semantics

Current sender defaults:

```text
position source: grip pose
rotation source: aim pose
```

Override options:

```bash
--position-pose grip   # default
--position-pose aim
--rotation-pose aim    # default
--rotation-pose grip
```

## Current Rotation Mapping

The VR leader uses `config/vr_teleop_config.yaml`.

Important defaults:

```yaml
vr_mapping:
  rotation_compose_order: "anchor_then_delta"
  invert_rotation_delta: true
```

Coordinate assumptions:

```text
OpenXR local: +X right, +Y up, -Z forward
Robot base:   +X forward, +Y left, +Z up
```

The configured matrix maps:

```text
OpenXR +X/right   -> robot -Y/right
OpenXR +Y/up      -> robot +Z/up
OpenXR -Z/forward -> robot +X/forward
```

## Real Hardware Startup

### 1. Follower PC

```bash
cd openarm_wifi_bimanual_teleop
sudo openarm-can-configure-socketcan can0 -fd
sudo openarm-can-configure-socketcan can1 -fd

./build/wifi_bimanual_follower \
  --interface <FOLLOWER_NET_IFACE> \
  --right-can can0 \
  --left-can can1 \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf \
  --right-port 50000 \
  --left-port 50001 \
  --control-rate-hz 500 \
  --watchdog-disable-ms 1000 \
  --publish-telemetry \
  --telemetry-ip <VR_LEADER_PC_IP> \
  --telemetry-port 51000 \
  --telemetry-rate-hz 100
```

### 2. VR Leader PC

```bash
cd openarm_wifi_bimanual_teleop
./build/wifi_vr_bimanual_leader \
  --follower-ip <FOLLOWER_PC_IP> \
  --right-urdf urdf/openarm_right.urdf \
  --left-urdf urdf/openarm_left.urdf \
  --config config/vr_teleop_config.yaml \
  --vr-port 54002 \
  --telemetry-port 51000 \
  --rate-hz 100
```

Useful options:

```bash
--ik-debug
--log-csv /tmp/vr_ik_debug.csv
--vr-mapping-test
--pose-ik
--position-ik
--set ik.orientation_weight=0.03
--set vr_mapping.rotation_compose_order=anchor_then_delta
--set vr_mapping.invert_rotation_delta=true
```

### 3. Quest/OpenXR Sender

```bash
cd ../quest2_openxr_logger
./build/openxr_relative_sender \
  --dest-ip <VR_LEADER_PC_IP> \
  --dest-port 54002 \
  --rotation-pose aim \
  --position-pose grip
```

Use `--print` for sender-side debug output.

## Operation

- Hold **Grip** to enable relative motion.
- Releasing Grip freezes the command and resets the controller anchor on the next press.
- The follower only receives enabled commands while the VR leader has fresh telemetry and valid IK output.
- Start with small movements and low scale values.

## Simulator / ROS 2 Bridge Variant

When using `openarm_udp_bridge`, start the simulator and bridge, then run:

```bash
./build/wifi_vr_bimanual_leader \
  --follower-ip 127.0.0.1 \
  --config config/vr_teleop_config.yaml
```

The bridge must send telemetry back to the VR leader at `--telemetry-port 51000`.

## Debugging the Mapping

Run the leader with:

```bash
--dry-run --vr-mapping-test --ik-debug --log-csv /tmp/ik_debug.csv
```

At startup, the leader prints the active OpenXR-to-robot axis mapping, whether rotation inversion is ON, and the compose order.
