#pragma once
#include "openarm_wifi_teleop/net/teleop_packet.hpp"
#include <cstdint>
#include <mutex>
#include <string>

namespace openarm_wifi_teleop {
namespace safety {

enum class SafetyState {
    DISABLED,
    WAITING_FOR_ENABLE,
    READY,
    ACTIVE,
    WARNING_TIMEOUT,
    HOLD,
    ESTOP,
    FAULT
};

struct SafetyConfig {
    bool require_enable = true;
    double watchdog_warning_ms = 20.0;
    double watchdog_hold_ms = 50.0;
    double watchdog_disable_ms = 100.0;
    double initial_pose_max_error_rad = 0.3;
    double max_target_delta_rad_per_cycle = 0.02;
    double max_joint_velocity_rad_s = 1.5;
    bool reject_out_of_order_packets = true;
};

class SafetyManager {
public:
    SafetyManager(const SafetyConfig& config, const std::string& arm_name = "UNKNOWN");
    
    void update(const net::TeleopPacket& latest_packet, double time_since_last_packet_ms);
    
    SafetyState get_state() const;
    void trigger_estop();
    void reset();

private:
    mutable std::mutex mutex_;
    SafetyConfig config_;
    SafetyState state_;
    std::string arm_name_;
    bool estop_triggered_;
};

}
}
