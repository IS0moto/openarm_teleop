#pragma once

#include <cstdint>

namespace openarm_wifi_teleop {
namespace net {

constexpr uint32_t TELEOP_MAGIC = 0x4F415754; // 'OAWT'
constexpr uint16_t TELEOP_VERSION = 1;

enum class ArmSide : uint8_t {
    RIGHT = 0,
    LEFT = 1
};

enum class ControlMode : uint8_t {
    UNILATERAL = 0
};

#pragma pack(push, 1)
struct TeleopPacket {
    uint32_t magic;          // 'OAWT'
    uint16_t version;        // 1
    uint16_t packet_size;

    uint32_t seq;
    uint64_t send_time_ns;

    uint8_t arm_side;        // 0: right, 1: left
    uint8_t mode;            // 0: unilateral
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
#pragma pack(pop)

} // namespace net
} // namespace openarm_wifi_teleop
