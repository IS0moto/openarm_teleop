#pragma once

#include <cstdint>

#pragma pack(push, 1)

/**
 * @brief VR Relative Teleop Packet V1
 * 
 * Sent from VR App (Quest2) to Robot Controller (Leader PC).
 * Contains relative displacement (delta) in robot space.
 */
struct VrRelativeTeleopPacketV1 {
    static constexpr uint32_t MAGIC = 0x54525651; // 'QVRT'
    static constexpr uint16_t VERSION = 1;

    uint32_t magic;          // 'QVRT'
    uint16_t version;        // 1
    uint16_t packet_size;

    uint32_t seq;
    uint64_t monotonic_time_ns;

    uint8_t mode;            // 1: relative
    uint8_t enabled;
    uint8_t estop;
    uint8_t reserved0;

    uint8_t left_grip;
    uint8_t right_grip;
    uint8_t left_trigger;
    uint8_t right_trigger;

    uint8_t left_recenter_event;
    uint8_t right_recenter_event;
    uint8_t reserved1;
    uint8_t reserved2;

    float left_delta_pos_robot[3];
    float right_delta_pos_robot[3];

    float left_delta_rot_robot[4];   // optional quaternion, identity for MVP
    float right_delta_rot_robot[4];  // optional quaternion, identity for MVP

    uint32_t crc32;
};

#pragma pack(pop)
