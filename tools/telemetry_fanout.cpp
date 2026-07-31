#include <arpa/inet.h>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

namespace {

volatile std::sig_atomic_t keep_running = 1;

void signal_handler(int) {
    keep_running = 0;
}

bool parse_endpoint(const std::string& text, std::string& ip, uint16_t& port) {
    const auto colon = text.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 >= text.size()) {
        return false;
    }

    ip = text.substr(0, colon);
    try {
        int parsed_port = std::stoi(text.substr(colon + 1));
        if (parsed_port <= 0 || parsed_port > 65535) {
            return false;
        }
        port = static_cast<uint16_t>(parsed_port);
    } catch (...) {
        return false;
    }
    return true;
}

bool make_addr(const std::string& ip, uint16_t port, sockaddr_in& addr) {
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    return inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) == 1;
}

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " [--bind-ip IP] [--listen-port PORT] "
        << "--target IP:PORT [--target IP:PORT ...]\n"
        << "\n"
        << "Example:\n"
        << "  " << argv0 << " --bind-ip 127.0.0.1 --listen-port 51001 "
        << "--target 127.0.0.1:51000 --target 127.0.0.1:51002\n";
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string bind_ip = "127.0.0.1";
    uint16_t listen_port = 51001;
    std::vector<sockaddr_in> targets;
    std::vector<std::string> target_labels;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--bind-ip" && i + 1 < argc) {
            bind_ip = argv[++i];
        } else if ((arg == "--listen-port" || arg == "--port") && i + 1 < argc) {
            int parsed_port = std::stoi(argv[++i]);
            if (parsed_port <= 0 || parsed_port > 65535) {
                std::cerr << "Invalid listen port: " << parsed_port << "\n";
                return 1;
            }
            listen_port = static_cast<uint16_t>(parsed_port);
        } else if (arg == "--target" && i + 1 < argc) {
            std::string ip;
            uint16_t port = 0;
            const std::string endpoint = argv[++i];
            if (!parse_endpoint(endpoint, ip, port)) {
                std::cerr << "Invalid target endpoint: " << endpoint << "\n";
                print_usage(argv[0]);
                return 1;
            }
            sockaddr_in addr;
            if (!make_addr(ip, port, addr)) {
                std::cerr << "Invalid target IP: " << ip << "\n";
                return 1;
            }
            targets.push_back(addr);
            target_labels.push_back(endpoint);
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown or incomplete argument: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    if (targets.empty()) {
        std::cerr << "At least one --target is required.\n";
        print_usage(argv[0]);
        return 1;
    }

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create UDP socket: " << std::strerror(errno) << "\n";
        return 1;
    }

    int reuse = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in bind_addr;
    if (!make_addr(bind_ip, listen_port, bind_addr)) {
        std::cerr << "Invalid bind IP: " << bind_ip << "\n";
        close(sockfd);
        return 1;
    }

    if (bind(sockfd, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        std::cerr << "Failed to bind " << bind_ip << ":" << listen_port
                  << ": " << std::strerror(errno) << "\n";
        close(sockfd);
        return 1;
    }

    std::cout << "Telemetry fanout listening on " << bind_ip << ":" << listen_port << "\n";
    for (const auto& label : target_labels) {
        std::cout << "  -> " << label << "\n";
    }

    std::vector<unsigned char> buffer(4096);
    uint64_t packets = 0;
    while (keep_running) {
        ssize_t len = recv(sockfd, buffer.data(), buffer.size(), 0);
        if (len < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "recv failed: " << std::strerror(errno) << "\n";
            break;
        }

        for (const auto& target : targets) {
            sendto(sockfd, buffer.data(), static_cast<size_t>(len), MSG_DONTWAIT,
                   reinterpret_cast<const sockaddr*>(&target), sizeof(target));
        }

        ++packets;
        // DEBUG-level heartbeat: hidden by default, shown with OPENARM_LOG_LEVEL=DEBUG.
        static const bool log_debug = [] {
            const char* e = std::getenv("OPENARM_LOG_LEVEL");
            return e && std::string(e) == "DEBUG";
        }();
        if (log_debug && packets % 100 == 0) {
            std::cout << "[DEBUG] Forwarded " << packets << " telemetry packets\n";
        }
    }

    close(sockfd);
    return 0;
}
