#include <gtest/gtest.h>
#include <cstring>
#include "openarm_wifi_teleop/net/packet_codec.hpp"

using namespace openarm_wifi_teleop::net;

TEST(FeedbackCodecTest, EncodeDecodeValid) {
    FeedbackPacket p;
    std::memset(&p, 0, sizeof(p));
    p.arm_tau[2] = -1.25;
    p.echo_cmd_send_time_ns = 123456789ULL;
    PacketCodec::encode(p);
    EXPECT_EQ(p.magic, FEEDBACK_MAGIC);
    EXPECT_EQ(p.packet_size, sizeof(FeedbackPacket));
    EXPECT_TRUE(PacketCodec::decode_and_validate(p));
    EXPECT_DOUBLE_EQ(p.arm_tau[2], -1.25);
    EXPECT_EQ(p.echo_cmd_send_time_ns, 123456789ULL);
}

TEST(FeedbackCodecTest, InvalidCRC) {
    FeedbackPacket p;
    std::memset(&p, 0, sizeof(p));
    PacketCodec::encode(p);
    p.arm_pos[0] += 0.001;  // payload changed after CRC
    EXPECT_FALSE(PacketCodec::decode_and_validate(p));
}

TEST(FeedbackCodecTest, TeleopPacketIsNotFeedback) {
    // A TeleopPacket reinterpreted as feedback must be rejected by magic/size.
    TeleopPacket t;
    std::memset(&t, 0, sizeof(t));
    PacketCodec::encode(t);
    FeedbackPacket p;
    std::memset(&p, 0, sizeof(p));
    std::memcpy(&p, &t, sizeof(t));
    EXPECT_FALSE(PacketCodec::decode_and_validate(p));
}

TEST(FeedbackCodecTest, HeaderSharesTeleopLayout) {
    // The arbiter validates both packet kinds with the same header parser.
    EXPECT_EQ(offsetof(FeedbackPacket, magic), offsetof(TeleopPacket, magic));
    EXPECT_EQ(offsetof(FeedbackPacket, version), offsetof(TeleopPacket, version));
    EXPECT_EQ(offsetof(FeedbackPacket, packet_size), offsetof(TeleopPacket, packet_size));
    EXPECT_EQ(offsetof(FeedbackPacket, seq), offsetof(TeleopPacket, seq));
    EXPECT_EQ(offsetof(FeedbackPacket, send_time_ns), offsetof(TeleopPacket, send_time_ns));
}
