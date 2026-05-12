#include "openarm_wifi_teleop/safety/rate_limiter.hpp"
#include <algorithm>
#include <cmath>

namespace openarm_wifi_teleop {
namespace safety {

RateLimiter::RateLimiter(double max_delta_per_cycle, double max_velocity_rad_s, double cycle_time_s)
    : max_delta_per_cycle_(max_delta_per_cycle), 
      max_velocity_rad_s_(max_velocity_rad_s),
      cycle_time_s_(cycle_time_s) {}

void RateLimiter::limit(const std::vector<double>& current_pos, std::vector<double>& target_pos) {
    if (current_pos.size() != target_pos.size()) return;

    double max_vel_delta = max_velocity_rad_s_ * cycle_time_s_;
    double effective_max_delta = std::min(max_delta_per_cycle_, max_vel_delta);

    for (size_t i = 0; i < target_pos.size(); ++i) {
        double delta = target_pos[i] - current_pos[i];
        if (delta > effective_max_delta) {
            target_pos[i] = current_pos[i] + effective_max_delta;
        } else if (delta < -effective_max_delta) {
            target_pos[i] = current_pos[i] - effective_max_delta;
        }
    }
}

}
}
