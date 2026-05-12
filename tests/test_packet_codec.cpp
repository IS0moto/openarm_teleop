#include <gtest/gtest.h>
#include "openarm_wifi_teleop/net/packet_codec.hpp"

using namespace openarm_wifi_teleop::net;

TEST(PacketCodecTest, EncodeDecodeValid) {
    TeleopPacket packet;
    PacketCodec::encode(packet);
    EXPECT_TRUE(PacketCodec::decode_and_validate(packet));
}

TEST(PacketCodecTest, InvalidMagic) {
    TeleopPacket packet;
    PacketCodec::encode(packet);
    packet.magic = 0;
    // We must recalculate CRC so validate fails because of magic, not CRC. Actually validate checks magic first.
    EXPECT_FALSE(PacketCodec::decode_and_validate(packet));
}

TEST(PacketCodecTest, InvalidCRC) {
    TeleopPacket packet;
    PacketCodec::encode(packet);
    packet.crc32 ^= 0xFF;
    EXPECT_FALSE(PacketCodec::decode_and_validate(packet));
}
