#pragma once
#include "openarm_wifi_teleop/net/teleop_packet.hpp"
#include <string>
#include <cstdint>
#include <functional>
#include <atomic>
#include <thread>

namespace openarm_wifi_teleop {
namespace net {

class UdpReceiver {
public:
    using PacketCallback = std::function<void(const TeleopPacket&)>;
    using RawPacketCallback = std::function<void(const void*, size_t)>;

    UdpReceiver(const std::string& bind_ip, uint16_t bind_port);
    ~UdpReceiver();

    bool start(PacketCallback callback);
    bool start_raw(RawPacketCallback callback);
    void stop();

    uint32_t get_received_count() const { return received_count_; }
    uint32_t get_invalid_count() const { return invalid_count_; }

private:
    void receive_loop();
    bool start_internal();

    int socket_fd_;
    std::string bind_ip_;
    uint16_t bind_port_;
    
    std::atomic<bool> running_;
    std::thread recv_thread_;
    PacketCallback callback_;
    RawPacketCallback raw_callback_;
    
    std::atomic<uint32_t> received_count_;
    std::atomic<uint32_t> invalid_count_;
};

}
}
