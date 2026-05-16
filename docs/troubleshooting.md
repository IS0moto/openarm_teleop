# Troubleshooting

## CAN Interface Is DOWN

Symptoms:

- No motor response.
- `candump can0` shows nothing.
- `ip link show can0` shows `state DOWN`.

Fix:

```bash
sudo openarm-can-configure-socketcan can0 -fd
sudo openarm-can-configure-socketcan can1 -fd
```

## CAN Is UP but Motors Do Not Respond

Check:

```bash
ip -details -statistics link show can0
candump can0 -t a
```

Common causes:

- CAN is not in CAN FD mode.
- Wrong CAN interface name.
- Power or cabling issue.
- Wrong motor IDs for the expected OpenArm configuration.

## Follower Stays in `WAIT_EN`

The follower is receiving no enabled command.

Check leader command line:

```bash
--enable
```

Check UDP ports:

```bash
ss -ulnp | grep -E '50000|50001'
```

Check IP/interface selection:

```bash
ip addr show
ping <Follower_IP>
```

## Follower Enters `HOLD` or `FAULT`

This means command packets are stale or missing.

Actions:

- Reduce `--rate-hz` from `500` to `250`.
- Use wired Ethernet or a dedicated Wi-Fi network.
- Increase first-test timeout with `--watchdog-disable-ms 1000`.
- Verify right and left ports are not swapped.

Status meanings:

| Status | Meaning |
| --- | --- |
| `WAIT_EN` | Waiting for enabled leader packet. |
| `READY` | Enable received, ready to control. |
| `ACTIVE` | Normal control. |
| `WARN` | Packet delay exceeded warning threshold. |
| `HOLD` | Holding last safe target. |
| `FAULT` | Timeout or safety fault; motors disabled. |
| `ESTOP` | Emergency stop. |

## VR Leader Shows `TEL: NO`

The VR leader requires telemetry from the follower or bridge.

For hardware follower, start it with:

```bash
--publish-telemetry --telemetry-ip <VR_LEADER_PC_IP> --telemetry-port 51000
```

For simulator/bridge, confirm the bridge sends telemetry to the VR leader.

## VR Leader Shows `VR: NO`

The Quest sender is not reaching the VR leader.

Check:

```bash
./build/openxr_relative_sender --dest-ip <VR_LEADER_PC_IP> --dest-port 54002 --print
ss -ulnp | grep 54002
```

## VR Motion Direction Is Wrong

Translation direction:

- Check `vr_mapping.right_translation_matrix` and `left_translation_matrix`.
- Run `wifi_vr_bimanual_leader --vr-mapping-test`.

Rotation direction:

- Current preferred setting is `invert_rotation_delta: true`.
- Current preferred compose order is `rotation_compose_order: "anchor_then_delta"`.
- Current preferred Quest pose sources are `position=grip`, `rotation=aim`.

Try sender comparisons:

```bash
--rotation-pose grip
--rotation-pose aim
--position-pose grip
--position-pose aim
```

## IK Frequently Fails

Actions:

- Release Grip, move to a more natural pose, and grip again.
- Lower `vr_mapping.position_scale_xyz`.
- Lower `vr_mapping.max_delta_m`.
- Increase `ik.damping`.
- Reduce `ik.orientation_weight`.
- Use `--position-ik` to test whether orientation is the issue.

Example:

```bash
./build/wifi_vr_bimanual_leader \
  --config config/vr_teleop_config.yaml \
  --set ik.damping=0.1 \
  --set ik.orientation_weight=0.03
```

## Useful Debug Commands

```bash
ip addr show
ip -details -statistics link show can0
candump can0 -t a
ss -ulnp | grep -E '50000|50001|51000|54002'
pkill -f wifi_bimanual
pkill -f wifi_vr_bimanual_leader
```
