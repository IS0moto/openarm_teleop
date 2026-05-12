#pragma once
#include "openarm_wifi_teleop/net/teleop_packet.hpp"
#include <vector>
#include <cstddef>

namespace openarm_wifi_teleop {
namespace net {

class PacketCodec {
public:
    static uint32_t calculate_crc32(const uint8_t* data, size_t length);
    static void encode(TeleopPacket& packet);
    static bool decode_and_validate(TeleopPacket& packet);
};

}
}
