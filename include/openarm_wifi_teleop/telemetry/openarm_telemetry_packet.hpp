#pragma once

#include <cstdint>

namespace openarm_wifi_teleop {
namespace telemetry {

constexpr uint32_t TELEMETRY_MAGIC = 0x5054414F; // 'OATP'

#pragma pack(push, 1)
struct OpenArmTelemetryPacketV1 {
    uint32_t magic;              // 'OATP'
    uint16_t version;            // 1
    uint16_t packet_size;

    uint32_t seq;
    uint64_t monotonic_time_ns;

    uint8_t robot_type;          // 1: openarm_bimanual
    uint8_t control_mode;        // 1: unilateral_wifi
    uint8_t enabled;
    uint8_t estop;

    uint8_t state_dim;           // 16
    uint8_t action_dim;          // 16
    uint8_t velocity_dim;        // 16 or 0
    uint8_t reserved0;

    uint8_t watchdog_state;
    uint8_t safety_state;
    uint8_t reserved1;
    uint8_t reserved2;

    float observation_state[16];
    float action[16];
    float observation_velocity[16];

    uint32_t crc32;
};
#pragma pack(pop)

} // namespace telemetry
} // namespace openarm_wifi_teleop
