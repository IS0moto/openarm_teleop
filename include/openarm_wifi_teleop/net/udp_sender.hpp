#pragma once
#include "openarm_wifi_teleop/net/teleop_packet.hpp"
#include <string>
#include <cstdint>

namespace openarm_wifi_teleop {
namespace net {

class UdpSender {
public:
    UdpSender(const std::string& target_ip, uint16_t target_port);
    UdpSender(const std::string& target_ip, uint16_t target_port, const std::string& local_ip, uint16_t local_port = 0);
    ~UdpSender();

    bool send(const TeleopPacket& packet);
    
    uint32_t get_sent_count() const { return sent_count_; }

private:
    int socket_fd_;
    std::string target_ip_;
    uint16_t target_port_;
    uint32_t sent_count_;
};

}
}
