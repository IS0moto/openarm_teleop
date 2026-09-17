#include "openarm_wifi_teleop/net/udp_receiver.hpp"
#include "openarm_wifi_teleop/net/packet_codec.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <cerrno>

namespace openarm_wifi_teleop {
namespace net {

UdpReceiver::UdpReceiver(const std::string& bind_ip, uint16_t bind_port)
    : bind_ip_(bind_ip), bind_port_(bind_port), running_(false), received_count_(0), invalid_count_(0) {
    
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        LOG_ERROR("Failed to create UDP socket on " << bind_port);
        return;
    }

    int reuse = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0) {
        LOG_WARN("Failed to set SO_REUSEADDR");
    }

    struct sockaddr_in bind_addr;
    std::memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(bind_port_);
    if (bind_ip_ == "0.0.0.0" || bind_ip_.empty()) {
        bind_addr.sin_addr.s_addr = INADDR_ANY;
    } else {
        inet_pton(AF_INET, bind_ip_.c_str(), &bind_addr.sin_addr);
    }

    if (bind(socket_fd_, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        LOG_ERROR("Failed to bind UDP socket to port " << bind_port_);
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

UdpReceiver::~UdpReceiver() {
    stop();
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

bool UdpReceiver::start(PacketCallback callback) {
    callback_ = callback;
    return start_internal();
}

bool UdpReceiver::start_raw(RawPacketCallback callback) {
    raw_callback_ = callback;
    return start_internal();
}

bool UdpReceiver::start_internal() {
    if (socket_fd_ < 0) return false;
    if (running_) return true;

    // Set non-blocking
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    running_ = true;
    recv_thread_ = std::thread(&UdpReceiver::receive_loop, this);
    
    LOG_INFO("Started UDP receiver on port " << bind_port_);
    return true;
}

void UdpReceiver::stop() {
    if (running_) {
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        LOG_INFO("Stopped UDP receiver on port " << bind_port_);
    }
}

void UdpReceiver::receive_loop() {
    uint8_t buffer[4096];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    while (running_) {
        // Block in poll() instead of sleeping 1 ms between polls: the sleep added
        // up to 1 ms of latency per hop, which matters for the bilateral loop.
        struct pollfd pfd;
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;
        int ready = poll(&pfd, 1, 10);
        if (ready <= 0) {
            if (ready < 0 && errno != EINTR) {
                LOG_ERROR("poll error: " << strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            continue;
        }

        ssize_t recv_bytes = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                      (struct sockaddr*)&client_addr, &client_len);

        if (recv_bytes > 0) {
            received_count_++;
            if (raw_callback_) {
                raw_callback_(buffer, recv_bytes);
            } else if (callback_) {
                if (recv_bytes == sizeof(TeleopPacket)) {
                    TeleopPacket packet;
                    std::memcpy(&packet, buffer, sizeof(TeleopPacket));
                    if (PacketCodec::decode_and_validate(packet)) {
                        callback_(packet);
                    } else {
                        invalid_count_++;
                    }
                } else {
                    invalid_count_++;
                }
            }
        } else if (recv_bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            LOG_ERROR("recvfrom error: " << strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

}
}
