#pragma once

#include "openarm_wifi_teleop/telemetry/openarm_telemetry_packet.hpp"
#include <string>
#include <vector>
#include <netinet/in.h>

namespace openarm_wifi_teleop {
namespace telemetry {

class OpenArmTelemetryPublisher {
public:
    OpenArmTelemetryPublisher(const std::string& ip, uint16_t port);
    ~OpenArmTelemetryPublisher();

    bool start();
    void stop();

    void publish(const OpenArmTelemetryPacketV1& packet);

private:
    std::string ip_;
    uint16_t port_;
    int sockfd_;
    struct sockaddr_in dest_addr_;
    uint32_t seq_;
    
    uint32_t calculate_crc32(const uint8_t* data, size_t length);
};

} // namespace telemetry
} // namespace openarm_wifi_teleop
