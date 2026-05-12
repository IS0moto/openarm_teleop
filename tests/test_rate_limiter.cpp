#include <gtest/gtest.h>
#include "openarm_wifi_teleop/safety/rate_limiter.hpp"

using namespace openarm_wifi_teleop::safety;

TEST(RateLimiterTest, LimitsVelocity) {
    // max 0.02 per cycle
    RateLimiter rl(0.02, 1.5, 1.0/500.0); // 1.5 * 0.002 = 0.003 is the effective limit
    std::vector<double> cur = {0.0};
    std::vector<double> tar = {0.1}; // large jump
    
    rl.limit(cur, tar);
    EXPECT_NEAR(tar[0], 0.003, 1e-6);
    
    tar[0] = -0.1;
    rl.limit(cur, tar);
    EXPECT_NEAR(tar[0], -0.003, 1e-6);
}
