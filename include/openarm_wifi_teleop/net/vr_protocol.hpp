#pragma once

#include <cstdint>

#pragma pack(push, 1)

namespace openarm_wifi_teleop {
namespace net {

struct VrRelativeTeleopPacketV2 {
    static constexpr uint32_t MAGIC = 0x3252564F; // 'OVR2'
    static constexpr uint16_t VERSION = 2;

    uint32_t magic;          // 'OVR2'
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

    uint32_t crc32;
};

} // namespace net
} // namespace openarm_wifi_teleop

#pragma pack(pop)
