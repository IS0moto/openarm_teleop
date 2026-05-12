#include <gtest/gtest.h>
#include "openarm_wifi_teleop/core/teleop_state_buffer.hpp"

using namespace openarm_wifi_teleop::core;
using namespace openarm_wifi_teleop::net;

TEST(StateBufferTest, GetLatest) {
    TeleopStateBuffer buf;
    TeleopPacket p1;
    p1.seq = 42;
    buf.update(p1);
    
    TeleopPacket p2;
    EXPECT_TRUE(buf.get_latest(p2));
    EXPECT_EQ(p2.seq, 42);
}
