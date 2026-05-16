# CLI and Configuration Reference

## `wifi_bimanual_leader`

Physical or mock leader. Sends right and left `TeleopPacket` streams to a follower.

| Option | Default | Description |
| --- | --- | --- |
| `--follower-ip` | `172.30.21.146` | Destination follower/bridge IP. |
| `--right-can` | `can0` | Right leader CAN interface. |
| `--left-can` | `can1` | Left leader CAN interface. |
| `--right-port` | `50000` | Right command UDP destination port. |
| `--left-port` | `50001` | Left command UDP destination port. |
| `--rate-hz` | `500` | Send/control loop rate. |
| `--right-urdf` | empty | Required in real mode. |
| `--left-urdf` | empty | Required in real mode. |
| `--bind-ip` | `0.0.0.0` | Local UDP bind IP. |
| `--interface` | empty | Auto-detect bind IP from interface. |
| `--local-port-r` | `0` | Local UDP source port for right stream. |
| `--local-port-l` | `0` | Local UDP source port for left stream. |
| `--enable` | off | Send enabled commands. |
| `--mock` | off | Do not use CAN; generate mock joint targets. |
| `--publish-telemetry` | off | Publish leader joint telemetry. |
| `--telemetry-ip` | `127.0.0.1` | Telemetry destination IP. |
| `--telemetry-port` | `51000` | Telemetry destination port. |
| `--telemetry-rate-hz` | `100` | Telemetry publish rate. |

## `wifi_bimanual_follower`

Receives right and left `TeleopPacket` streams and drives follower arms.

| Option | Default | Description |
| --- | --- | --- |
| `--right-can` | `can0` | Right follower CAN interface. |
| `--left-can` | `can1` | Left follower CAN interface. |
| `--right-port` | `50000` | Right command UDP listen port. |
| `--left-port` | `50001` | Left command UDP listen port. |
| `--control-rate-hz` | `500` | Follower control loop rate. |
| `--right-urdf` | empty | Required in real mode. |
| `--left-urdf` | empty | Required in real mode. |
| `--bind-ip` | `0.0.0.0` | UDP bind IP. |
| `--interface` | empty | Auto-detect bind IP from interface. |
| `--watchdog-hold-ms` | safety default | Time before HOLD state. |
| `--watchdog-disable-ms` | safety default | Time before disabling/FAULT. |
| `--publish-telemetry` | off | Publish measured/target state telemetry. |
| `--telemetry-ip` | `127.0.0.1` | Telemetry destination IP. |
| `--telemetry-port` | `51000` | Telemetry destination port. |
| `--telemetry-rate-hz` | `100` | Telemetry publish rate. |
| `--telemetry-log-stats` | off | Print telemetry stats. |
| `--mock` | off | Do not use CAN; run receiver/safety loop only. |

## `wifi_vr_bimanual_leader`

Receives Quest VR deltas, consumes follower telemetry, solves IK, and sends follower joint targets.

| Option | Default | Description |
| --- | --- | --- |
| `--follower-ip` | `127.0.0.1` | Destination follower/bridge IP. |
| `--vr-port` | `54002` | UDP listen port for Quest packets. |
| `--telemetry-port` | `51000` | UDP listen port for follower telemetry. |
| `--rate-hz` | `100` | IK/send loop rate. |
| `--right-urdf` | `urdf/openarm_right.urdf` | Right arm URDF. |
| `--left-urdf` | `urdf/openarm_left.urdf` | Left arm URDF. |
| `--config` | `config/vr_teleop_config.yaml` | VR mapping/IK config. |
| `--ik-debug` | off | Print and enable IK debug logging. |
| `--ik-debug-rate-hz` | config | Override debug log rate. |
| `--ik-debug-burst-sec` | config | Override burst duration after anchor update. |
| `--log-csv` | empty | Write IK debug CSV. |
| `--max-test-sec` | `0` | Stop after N seconds; useful with `--dry-run`. |
| `--dry-run` | off | Do not enable outgoing robot commands. |
| `--vr-mapping-test` | off | Print mapped delta diagnostics. |
| `--pose-ik` | config | Use pose IK. |
| `--position-ik` | config | Ignore orientation in IK target. |
| `--set key=value` | none | Override supported config fields. |

Supported `--set` keys:

```text
ik.orientation_weight
ik.position_weight
ik.damping
ik.max_dq_norm
ik.mode
human_model.enabled
swivel_prior.enabled
swivel_prior.apply_to_ik
swivel_prior.log_only
vr_mapping.rotation_compose_order
vr_mapping.invert_rotation_delta
debug.ik_debug_rate_hz
debug.ik_debug_burst_sec
```

## `openxr_relative_sender`

Lives in `../quest2_openxr_logger`. Sends Quest controller relative deltas to `wifi_vr_bimanual_leader`.

| Option | Default | Description |
| --- | --- | --- |
| `--config` | `configs/vr_relative_mapping.yaml` | UDP destination and sender rate config. |
| `--print` | off | Print packet/debug information. |
| `--dest-ip`, `--leader-ip` | config | Override destination IP. |
| `--dest-port`, `--port` | config | Override destination port. |
| `--log-csv` | empty | Log raw controller deltas. |
| `--position-pose grip|aim` | `grip` | Pose source for relative translation. |
| `--rotation-pose grip|aim` | `aim` | Pose source for relative rotation. |
| `--position-aim-pose` / `--position-grip-pose` | grip | Shortcut toggles. |
| `--rotation-aim-pose` / `--rotation-grip-pose` | aim | Shortcut toggles. |

## `wifi_single_leader` / `wifi_single_follower`

Single-arm bring-up variants.

Leader:

```text
--follower-ip 127.0.0.1
--can can0
--port 50000
--rate-hz 500
--urdf <path>
--enable
--mock
```

Follower:

```text
--can can0
--port 50000
--control-rate-hz 500
--urdf <path>
--watchdog-hold-ms <ms>
--watchdog-disable-ms <ms>
--mock
```

## VR Config Highlights

File: `config/vr_teleop_config.yaml`

```yaml
vr_mapping:
  position_scale_xyz: [0.8, 0.8, 0.8]
  max_delta_m: 0.35
  rotation_compose_order: "anchor_then_delta"
  invert_rotation_delta: true
```

The leader prints the active basis mapping, rotation inversion, and compose order at startup.
