#include "openarm_wifi_teleop/net/packet_codec.hpp"
#include "openarm_wifi_teleop/utils/time.hpp"
#include <cstring>

namespace openarm_wifi_teleop {
namespace net {

uint32_t PacketCodec::calculate_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
        }
    }
    return ~crc;
}

void PacketCodec::encode(TeleopPacket& packet) {
    packet.magic = TELEOP_MAGIC;
    packet.version = TELEOP_VERSION;
    packet.packet_size = sizeof(TeleopPacket);
    packet.send_time_ns = openarm_wifi_teleop::utils::system_now_ns();
    packet.crc32 = 0; // Set to 0 before calculation
    
    uint32_t crc = calculate_crc32(reinterpret_cast<const uint8_t*>(&packet), sizeof(TeleopPacket) - sizeof(uint32_t));
    packet.crc32 = crc;
}

bool PacketCodec::decode_and_validate(TeleopPacket& packet) {
    if (packet.magic != TELEOP_MAGIC) {
        return false;
    }
    if (packet.version != TELEOP_VERSION) {
        return false;
    }
    if (packet.packet_size != sizeof(TeleopPacket)) {
        return false;
    }
    
    uint32_t received_crc = packet.crc32;
    packet.crc32 = 0; // Clear for calculation
    uint32_t calculated_crc = calculate_crc32(reinterpret_cast<const uint8_t*>(&packet), sizeof(TeleopPacket) - sizeof(uint32_t));
    packet.crc32 = received_crc; // Restore
    
    return calculated_crc == received_crc;
}

}
}
