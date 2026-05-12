#include "openarm_wifi_teleop/net/udp_sender.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

namespace openarm_wifi_teleop {
namespace net {

UdpSender::UdpSender(const std::string& target_ip, uint16_t target_port)
    : target_ip_(target_ip), target_port_(target_port), sent_count_(0) {
    
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        LOG_ERROR("Failed to create UDP socket for " << target_ip << ":" << target_port);
    }
}

UdpSender::~UdpSender() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

bool UdpSender::send(const TeleopPacket& packet) {
    if (socket_fd_ < 0) return false;

    struct sockaddr_in target_addr;
    std::memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(target_port_);
    if (inet_pton(AF_INET, target_ip_.c_str(), &target_addr.sin_addr) <= 0) {
        LOG_ERROR("Invalid target IP: " << target_ip_);
        return false;
    }

    ssize_t sent_bytes = sendto(socket_fd_, &packet, sizeof(TeleopPacket), 0,
                                (struct sockaddr*)&target_addr, sizeof(target_addr));
                                
    if (sent_bytes == sizeof(TeleopPacket)) {
        sent_count_++;
        return true;
    } else {
        LOG_WARN("Failed to send full packet to " << target_ip_ << ":" << target_port_);
        return false;
    }
}

}
}
