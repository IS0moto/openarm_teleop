#include "openarm_wifi_teleop/safety/bilateral_gate.hpp"

#include <sstream>

namespace openarm_wifi_teleop {
namespace safety {

std::string BilateralGateResult::reasons_joined(const std::string& sep) const {
    std::ostringstream os;
    for (size_t i = 0; i < reasons.size(); ++i) {
        if (i) os << sep;
        os << reasons[i];
    }
    return os.str();
}

BilateralGateResult BilateralGate::check_link(const utils::LinkInfo& link) const {
    BilateralGateResult r;
    if (!link.resolved) {
        r.reasons.push_back("link unresolved: " + link.error);
        r.ok = false;
        return r;
    }
    if (link.is_loopback) {
        if (!config_.allow_loopback) {
            r.reasons.push_back("peer is reached over loopback (set AllowLoopback for tests)");
        }
    } else if (config_.require_wired) {
        if (link.is_wireless) {
            r.reasons.push_back("interface " + link.ifname + " is wireless");
        }
        if (!link.has_device) {
            r.reasons.push_back("interface " + link.ifname + " is virtual (no physical device)");
        }
        if (!link.carrier) {
            r.reasons.push_back("interface " + link.ifname + " has no carrier");
        }
        if (link.speed_mbps < config_.min_link_speed_mbps) {
            std::ostringstream os;
            os << "interface " << link.ifname << " speed " << link.speed_mbps
               << " Mbps < " << config_.min_link_speed_mbps << " Mbps";
            r.reasons.push_back(os.str());
        }
    }
    r.ok = r.reasons.empty();
    return r;
}

BilateralGateResult BilateralGate::check_feedback(const std::string& arm_name,
                                                  const core::FeedbackStateBuffer::Stats& s,
                                                  const net::FeedbackPacket* latest) const {
    BilateralGateResult r;
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(2);

    if (!s.has_data || latest == nullptr) {
        r.reasons.push_back(arm_name + ": no feedback received");
        r.ok = false;
        return r;
    }
    if (s.age_ms > config_.max_feedback_age_ms) {
        os.str(""); os << arm_name << ": feedback age " << s.age_ms << " ms > " << config_.max_feedback_age_ms;
        r.reasons.push_back(os.str());
    }
    if (s.rate_hz < config_.min_feedback_rate_hz) {
        os.str(""); os << arm_name << ": feedback rate " << s.rate_hz << " Hz < " << config_.min_feedback_rate_hz;
        r.reasons.push_back(os.str());
    }
    if (s.rtt_p95_ms < 0.0) {
        r.reasons.push_back(arm_name + ": no round-trip samples (follower not echoing commands)");
    } else if (s.rtt_p95_ms > config_.max_rtt_p95_ms) {
        os.str(""); os << arm_name << ": rtt p95 " << s.rtt_p95_ms << " ms > " << config_.max_rtt_p95_ms;
        r.reasons.push_back(os.str());
    }
    if (s.loss_percent > config_.max_loss_percent) {
        os.str(""); os << arm_name << ": loss " << s.loss_percent << " % > " << config_.max_loss_percent;
        r.reasons.push_back(os.str());
    }
    if (config_.require_follower_link_ok && latest->link_ok == 0) {
        r.reasons.push_back(arm_name + ": follower reports its own link is not wired/fast enough");
    }
    if (latest->estop) {
        r.reasons.push_back(arm_name + ": follower in ESTOP");
    }
    r.ok = r.reasons.empty();
    return r;
}

}
}
