# OpenArm Telemetry Protocol V1

This document specifies the fixed-length binary protocol used by `openarm_wifi_bimanual_teleop` to broadcast the actual robot state and target references.

## Packet Layout

The telemetry is sent as a raw UDP datagram. All fields are in Little-Endian byte order. The total length of `OpenArmTelemetryPacketV1` is 164 bytes.

```c
#pragma pack(push, 1)
struct OpenArmTelemetryPacketV1 {
    uint32_t magic;              // Always 0x5054414F ('OATP')
    uint16_t version;            // 1
    uint16_t packet_size;        // Size of this struct (164 bytes)

    uint32_t seq;                // Monotonically increasing sequence number
    uint64_t monotonic_time_ns;  // System monotonic time on the sender

    uint8_t robot_type;          // 1: openarm_bimanual
    uint8_t control_mode;        // 1: unilateral_wifi
    uint8_t enabled;             // 1: motors are enabled, 0: disabled
    uint8_t estop;               // 1: estop triggered, 0: safe

    uint8_t state_dim;           // Number of dimensions for state (16)
    uint8_t action_dim;          // Number of dimensions for action (16)
    uint8_t velocity_dim;        // Number of dimensions for velocity (16)
    uint8_t reserved0;

    uint8_t watchdog_state;      // Sender's internal watchdog state
    uint8_t safety_state;        // Sender's internal safety state
    uint8_t reserved1;
    uint8_t reserved2;

    float observation_state[16]; // Actual measured joint positions
    float action[16];            // Safe/limited target joint positions sent to control
    float observation_velocity[16]; // Actual measured joint velocities

    uint32_t crc32;              // CRC32 of the packet (excluding the crc32 field itself)
};
#pragma pack(pop)
```

## State/Action Dimensions

The array order is fixed to exactly 16 dimensions for the bimanual setup:

- `index 0`  to `index 6`: Right Arm Joint 1 - 7
- `index 7`:               Right Gripper
- `index 8`  to `index 14`: Left Arm Joint 1 - 7
- `index 15`:              Left Gripper

## Validation (Receiving)

When implementing a receiver:
1. Ensure the received payload length equals `packet_size`.
2. Check `magic == 0x5054414F`.
3. Check `version == 1`.
4. (Optional but recommended) Calculate the CRC32 of the first `length - 4` bytes and compare it to the `crc32` field.
5. Track the `seq` number to drop out-of-order or duplicate packets.
6. Track `monotonic_time_ns` to filter out stale data (e.g. data older than 100ms).
