#include <gtest/gtest.h>
#include "openarm_wifi_teleop/core/teleop_state_buffer.hpp"

using namespace openarm_wifi_teleop::core;
using namespace openarm_wifi_teleop::net;

TEST(SeqLossDetection, InOrder) {
    TeleopStateBuffer buf;
    TeleopPacket p1, p2;
    p1.seq = 1;
    p2.seq = 2;
    buf.update(p1);
    buf.update(p2);
    EXPECT_EQ(buf.get_lost_packet_count(), 0);
}

TEST(SeqLossDetection, OutOfOrderRejected) {
    TeleopStateBuffer buf;
    TeleopPacket p1, p2;
    p1.seq = 5;
    p2.seq = 2;
    buf.update(p1);
    buf.update(p2); // Should be rejected
    
    TeleopPacket latest;
    buf.get_latest(latest);
    EXPECT_EQ(latest.seq, 5);
}

TEST(SeqLossDetection, DetectLoss) {
    TeleopStateBuffer buf;
    TeleopPacket p1, p2;
    p1.seq = 1;
    p2.seq = 5; // Missed 2, 3, 4
    buf.update(p1);
    buf.update(p2);
    EXPECT_EQ(buf.get_lost_packet_count(), 3);
}
