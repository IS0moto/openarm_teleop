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
    uint8_t mode;            // 0 unilateral, 1 bilateral (leader requests feedback)
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

## FeedbackPacket (bilateral)

Used by:

```text
wifi_bimanual_follower -> Control Arbiter (50400/50401) -> wifi_bimanual_leader (50500/50501)
```

Header: `include/openarm_wifi_teleop/net/feedback_packet.hpp`. 340 bytes. The
first 24 bytes have the same layout as `TeleopPacket` so the Arbiter validates
both with one header parser. The Arbiter forwards feedback only while its mode
is `leader`.

```c
struct FeedbackPacket {
    uint32_t magic;          // 0x4246414F ('OAFB')
    uint16_t version;        // 1
    uint16_t packet_size;    // 340

    uint32_t seq;
    uint64_t send_time_ns;   // follower system clock

    uint8_t arm_side;        // 0 right, 1 left
    uint8_t mode;            // ControlMode echoed from the applied command
    uint8_t enabled;         // 1: follower safety state is ACTIVE
    uint8_t estop;

    uint8_t arm_dof;
    uint8_t hand_dof;
    uint8_t safety_state;    // safety::SafetyState
    uint8_t link_ok;         // 1: follower-side wired/speed check passed

    uint32_t echo_cmd_seq;           // seq of the newest applied TeleopPacket
    uint64_t echo_cmd_send_time_ns;  // its send_time_ns (leader clock) -> RTT at the leader
    uint64_t cmd_hold_ns;            // follower-side age of that command when sent

    double arm_pos[8];
    double arm_vel[8];
    double arm_tau[8];       // measured joint torque [Nm]
    double hand_pos[4];
    double hand_vel[4];
    double hand_tau[4];      // measured gripper motor torque [Nm]

    uint32_t crc32;
};
```

Validation is the same as `TeleopPacket` (magic, version, size, CRC32, seq).
The leader computes `rtt = system_now_ns - echo_cmd_send_time_ns` on its own
clock; subtract `cmd_hold_ns` for the pure network + Arbiter delay.

See [bilateral_design.md](bilateral_design.md) for the gate that decides when
this feedback may be used.

## VR Relative Teleop Packet V2

Used by:

```text
openxr_relative_sender -> wifi_vr_bimanual_leader
```

Header: `include/openarm_wifi_teleop/net/vr_protocol.hpp`

```c
struct VrRelativeTeleopPacketV2 {
    static constexpr uint32_t MAGIC = 0x3252564F; // 0x3252564F ('OVR2')
    static constexpr uint16_t VERSION = 2;

    uint32_t magic;          // 'OVR2'
    uint16_t version;        // 2
    uint16_t packet_size;    // sizeof(VrRelativeTeleopPacketV2) == 256 bytes

    uint32_t seq;
    double timestamp_sec;

    // HMD World Poses
    float hmd_abs_pos_world[3];
    float hmd_abs_quat_world_xyzw[4];

    // Left Arm Data
    float left_delta_pos_world[3];
    float left_delta_rot_world_xyzw[4];
    float left_delta_pos_hmd[3];
    float left_delta_rot_hmd_xyzw[4];
    float left_abs_pos_world[3];
    float left_abs_rot_world_xyzw[4];
    float left_grip;
    float left_trigger;
    float left_thumbstick[2];
    uint8_t left_buttons;    // Bitmasks: 0x01 X, 0x02 Y, 0x04 Menu

    // Right Arm Data
    float right_delta_pos_world[3];
    float right_delta_rot_world_xyzw[4];
    float right_delta_pos_hmd[3];
    float right_delta_rot_hmd_xyzw[4];
    float right_abs_pos_world[3];
    float right_abs_rot_world_xyzw[4];
    float right_grip;
    float right_trigger;
    float right_thumbstick[2];
    uint8_t right_buttons;   // Bitmasks: 0x01 A, 0x02 B, 0x04 Menu

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
