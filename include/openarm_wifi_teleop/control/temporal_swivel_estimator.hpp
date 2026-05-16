#pragma once

#include <Eigen/Dense>
#include <algorithm>

namespace openarm_wifi_teleop {
namespace control {

struct SwivelInput {
    Eigen::Vector3d shoulder_pos;
    Eigen::Vector3d wrist_pos;
    Eigen::Quaterniond wrist_orientation;
    double previous_swivel;
    double dt;
    bool right_arm;
};

class TemporalSwivelEstimator {
public:
    struct Config {
        bool enabled = true;
        double neutral_gain = 0.2;
        double continuity_gain = 0.7;
        double wrist_orientation_gain = 0.1;
        double max_swivel_rate_rad_s = 1.0;
        double neutral_swivel_rad = 0.0;
    };

    TemporalSwivelEstimator(const Config& cfg) : cfg_(cfg), current_swivel_(cfg.neutral_swivel_rad) {}

    void reset(double neutral_swivel_rad) {
        current_swivel_ = neutral_swivel_rad;
    }

    double update(const SwivelInput& input) {
        if (!cfg_.enabled) return cfg_.neutral_swivel_rad;

        // 1. Simple heuristic for desired swivel
        // For now, we prioritize continuity and neutral posture.
        // Wrist orientation gain can be added if we have a good mapping.
        
        double swivel_desired = 
            cfg_.neutral_gain * cfg_.neutral_swivel_rad +
            cfg_.continuity_gain * current_swivel_;
        
        // 2. Apply rate limit
        double max_delta = cfg_.max_swivel_rate_rad_s * input.dt;
        double delta = swivel_desired - current_swivel_;
        delta = std::max(-max_delta, std::min(max_delta, delta));
        
        current_swivel_ += delta;
        return current_swivel_;
    }

private:
    Config cfg_;
    double current_swivel_;
};

} // namespace control
} // namespace openarm_wifi_teleop
