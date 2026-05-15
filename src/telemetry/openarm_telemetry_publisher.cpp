#include "openarm_wifi_teleop/telemetry/openarm_telemetry_publisher.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace openarm_wifi_teleop {
namespace telemetry {

OpenArmTelemetryPublisher::OpenArmTelemetryPublisher(const std::string& ip, uint16_t port)
    : ip_(ip), port_(port), sockfd_(-1), seq_(0) {
    memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(port_);
    if (inet_pton(AF_INET, ip_.c_str(), &dest_addr_.sin_addr) <= 0) {
        LOG_ERROR("Invalid telemetry IP address: " << ip_);
    }
}

OpenArmTelemetryPublisher::~OpenArmTelemetryPublisher() {
    stop();
}

bool OpenArmTelemetryPublisher::start() {
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        LOG_ERROR("Failed to create UDP socket for telemetry");
        return false;
    }
    LOG_INFO("Telemetry publisher started: " << ip_ << ":" << port_);
    return true;
}

void OpenArmTelemetryPublisher::stop() {
    if (sockfd_ >= 0) {
        close(sockfd_);
        sockfd_ = -1;
    }
}

uint32_t OpenArmTelemetryPublisher::calculate_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

void OpenArmTelemetryPublisher::publish(const OpenArmTelemetryPacketV1& packet) {
    if (sockfd_ < 0) return;

    OpenArmTelemetryPacketV1 to_send = packet;
    to_send.magic = TELEMETRY_MAGIC;
    to_send.version = 1;
    to_send.packet_size = sizeof(OpenArmTelemetryPacketV1);
    to_send.seq = seq_++;
    to_send.crc32 = 0;

    uint32_t crc = calculate_crc32(reinterpret_cast<const uint8_t*>(&to_send), sizeof(to_send) - sizeof(uint32_t));
    to_send.crc32 = crc;

    ssize_t sent = sendto(sockfd_, &to_send, sizeof(to_send), MSG_DONTWAIT,
                          (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
    
    if (sent < 0) {
        // Just drop the packet if sending fails to avoid blocking the control loop
        // It's expected if network is busy or unreachable.
    }
}

} // namespace telemetry
} // namespace openarm_wifi_teleop
