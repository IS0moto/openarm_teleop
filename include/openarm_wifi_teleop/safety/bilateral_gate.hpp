#pragma once

#include "openarm_wifi_teleop/core/feedback_state_buffer.hpp"
#include "openarm_wifi_teleop/net/feedback_packet.hpp"
#include "openarm_wifi_teleop/utils/link_check.hpp"

#include <string>
#include <vector>

namespace openarm_wifi_teleop {
namespace safety {

// Conditions that must all hold before bilateral feedback may be applied to
// the leader arms, and that are re-checked continuously while engaged.
struct BilateralGateConfig {
    bool require_wired = true;       // reject wireless and virtual interfaces
    bool allow_loopback = false;     // permit 127.0.0.1 peers (mock / single-PC tests)
    int min_link_speed_mbps = 100;   // sysfs speed of the interface toward the peer
    double min_feedback_rate_hz = 400.0;
    double max_feedback_age_ms = 20.0;
    double max_rtt_p95_ms = 8.0;
    double max_loss_percent = 1.0;
    bool require_follower_link_ok = true;  // follower's own wired check must pass
};

struct BilateralGateResult {
    bool ok = false;
    std::vector<std::string> reasons;  // empty when ok

    std::string reasons_joined(const std::string& sep = "; ") const;
};

class BilateralGate {
public:
    explicit BilateralGate(const BilateralGateConfig& config) : config_(config) {}

    // Local link only (checked once at enable time and periodically).
    BilateralGateResult check_link(const utils::LinkInfo& link) const;

    // Feedback quality for one arm. `latest` may be null when nothing received yet.
    BilateralGateResult check_feedback(const std::string& arm_name,
                                       const core::FeedbackStateBuffer::Stats& stats,
                                       const net::FeedbackPacket* latest) const;

    const BilateralGateConfig& config() const { return config_; }

private:
    BilateralGateConfig config_;
};

}
}
