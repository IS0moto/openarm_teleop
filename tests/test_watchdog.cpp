#include <gtest/gtest.h>
#include "openarm_wifi_teleop/safety/watchdog.hpp"

using namespace openarm_wifi_teleop::safety;

TEST(WatchdogTest, CheckTransitions) {
    Watchdog wd(20.0, 50.0, 100.0);
    EXPECT_EQ(wd.check(10.0), Watchdog::Status::OK);
    EXPECT_EQ(wd.check(25.0), Watchdog::Status::WARNING);
    EXPECT_EQ(wd.check(60.0), Watchdog::Status::HOLD);
    EXPECT_EQ(wd.check(120.0), Watchdog::Status::DISABLE);
}
