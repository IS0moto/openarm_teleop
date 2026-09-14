#pragma once

#include <cstdint>

namespace openarm_wifi_teleop {
namespace net {

// Follower -> leader state feedback for bilateral teleop.
//
// Header layout (magic/version/packet_size/seq/send_time_ns) is identical to
// TeleopPacket so the arbiter can validate both with the same code. The
// echo_* fields carry the leader-side send_time_ns/seq of the newest command
// the follower had applied when this packet was built, which lets the leader
// measure the full command->feedback round trip on its own clock.
constexpr uint32_t FEEDBACK_MAGIC = 0x4246414F; // 'OAFB'
constexpr uint16_t FEEDBACK_VERSION = 1;

#pragma pack(push, 1)
struct FeedbackPacket {
    uint32_t magic;          // 'OAFB'
    uint16_t version;        // 1
    uint16_t packet_size;

    uint32_t seq;
    uint64_t send_time_ns;   // follower system clock (not comparable across hosts)

    uint8_t arm_side;        // 0: right, 1: left
    uint8_t mode;            // ControlMode echoed from the applied command
    uint8_t enabled;         // 1: follower is tracking commands (safety ACTIVE)
    uint8_t estop;           // 1: follower is in ESTOP

    uint8_t arm_dof;
    uint8_t hand_dof;
    uint8_t safety_state;    // safety::SafetyState of this arm
    uint8_t link_ok;         // 1: follower-side link check passed (wired, speed ok)

    uint32_t echo_cmd_seq;           // seq of the newest applied TeleopPacket
    uint64_t echo_cmd_send_time_ns;  // send_time_ns of that TeleopPacket (leader clock)
    uint64_t cmd_hold_ns;            // follower-side age of that command when this was sent

    double arm_pos[8];       // measured joint position [rad]
    double arm_vel[8];       // measured joint velocity [rad/s]
    double arm_tau[8];       // measured joint torque [Nm] (motor feedback)
    double hand_pos[4];
    double hand_vel[4];
    double hand_tau[4];

    uint32_t crc32;
};
#pragma pack(pop)

static_assert(sizeof(FeedbackPacket) == 340, "FeedbackPacket layout changed; update protocol docs");

} // namespace net
} // namespace openarm_wifi_teleop
