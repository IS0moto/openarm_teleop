# UDP Protocols

This repo uses fixed-size binary UDP datagrams. The implementation assumes little-endian hosts.

## TeleopPacket

Used by:

```text
wifi_bimanual_leader -> wifi_bimanual_follower
wifi_vr_bimanual_leader -> wifi_bimanual_follower / bridge
```

Header: `include/openarm_wifi_teleop/net/teleop_packet.hpp`

```c
struct TeleopPacket {
    uint32_t magic;          // 0x4F415754 ('OAWT')
    uint16_t version;        // 1
    uint16_t packet_size;

    uint32_t seq;
    uint64_t send_time_ns;

    uint8_t arm_side;        // 0 right, 1 left
    uint8_t mode;            // 0 unilateral
    uint8_t enable;          // 0/1
    uint8_t estop;           // 0/1

    uint8_t arm_dof;
    uint8_t hand_dof;
    uint8_t reserved0;
    uint8_t reserved1;

    double arm_pos[8];
    double arm_vel[8];
    double hand_pos[4];
    double hand_vel[4];

    uint32_t crc32;
};
```

Ports:

| Stream | Default |
| --- | --- |
| Right arm command | `50000` |
| Left arm command | `50001` |

Validation:

- `magic == 0x4F415754`
- `version == 1`
- `packet_size == sizeof(TeleopPacket)`
- CRC32 must match.
- Out-of-order packets are rejected by `TeleopStateBuffer`.
- Sequence gaps increment the loss counter.

## VR Relative Teleop Packet V2

Used by:

```text
openxr_relative_sender -> wifi_vr_bimanual_leader
```

Header: `include/openarm_wifi_teleop/net/vr_protocol.hpp`

```c
struct VrRelativeTeleopPacketV2 {
    uint32_t magic;          // 0x3252564F ('OVR2')
    uint16_t version;        // 2
    uint16_t packet_size;

    uint32_t seq;
    double timestamp_sec;

    float right_delta_pos_openxr[3];
    float right_delta_rot_openxr_xyzw[4];

    float left_delta_pos_openxr[3];
    float left_delta_rot_openxr_xyzw[4];

    float hmd_pos_openxr[3];
    float hmd_quat_openxr_xyzw[4];

    float right_grip;
    float left_grip;
    float right_trigger;
    float left_trigger;

    uint8_t right_buttons;   // Bit 0: A, Bit 1: B, Bit 2: Menu
    uint8_t left_buttons;    // Bit 0: X, Bit 1: Y, Bit 2: Menu
    uint16_t reserved;

    uint32_t crc32;
};
```

Default port:

```text
54002
```

V2 sends raw OpenXR deltas. The robot-space mapping happens in `wifi_vr_bimanual_leader`.

## Telemetry

Telemetry uses `OpenArmTelemetryPacketV1`, documented in [telemetry_protocol.md](telemetry_protocol.md).
