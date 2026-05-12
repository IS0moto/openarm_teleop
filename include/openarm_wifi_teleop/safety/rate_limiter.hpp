#pragma once
#include <vector>
#include <cstdint>

namespace openarm_wifi_teleop {
namespace safety {

class RateLimiter {
public:
    RateLimiter(double max_delta_per_cycle, double max_velocity_rad_s, double cycle_time_s);

    void limit(const std::vector<double>& current_pos, std::vector<double>& target_pos);

private:
    double max_delta_per_cycle_;
    double max_velocity_rad_s_;
    double cycle_time_s_;
};

}
}
