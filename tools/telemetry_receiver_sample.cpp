#include "openarm_wifi_teleop/telemetry/openarm_telemetry_packet.hpp"
#include <iostream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

using namespace openarm_wifi_teleop::telemetry;

int main(int argc, char** argv) {
    std::string bind_ip = "0.0.0.0";
    uint16_t port = 51000;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--bind-ip" && i + 1 < argc) bind_ip = argv[++i];
        else if (arg == "--port" && i + 1 < argc) port = std::stoi(argv[++i]);
    }

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket\n";
        return 1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, bind_ip.c_str(), &addr.sin_addr) <= 0) {
        std::cerr << "Invalid IP address\n";
        return 1;
    }

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Bind failed\n";
        return 1;
    }

    std::cout << "Listening for telemetry on " << bind_ip << ":" << port << std::endl;

    OpenArmTelemetryPacketV1 packet;
    while (true) {
        ssize_t len = recv(sockfd, &packet, sizeof(packet), 0);
        if (len == sizeof(OpenArmTelemetryPacketV1)) {
            if (packet.magic == TELEMETRY_MAGIC) {
                std::cout << "Received packet seq: " << packet.seq 
                          << " state_dim: " << (int)packet.state_dim 
                          << " [R1 pos: " << packet.observation_state[0] << "]"
                          << std::endl;
            } else {
                std::cerr << "Invalid magic number\n";
            }
        }
    }

    close(sockfd);
    return 0;
}
