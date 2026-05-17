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
    uint8_t left_buttons;

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
    uint8_t right_buttons;

    uint16_t reserved;
    uint32_t crc32;
};

// Button Bitmasks
static constexpr uint8_t VR_BUTTON_A    = 0x01;
static constexpr uint8_t VR_BUTTON_B    = 0x02;
static constexpr uint8_t VR_BUTTON_X    = 0x01;
static constexpr uint8_t VR_BUTTON_Y    = 0x02;
static constexpr uint8_t VR_BUTTON_MENU = 0x04;

} // namespace net
} // namespace openarm_wifi_teleop

#pragma pack(pop)
