#include "openarm_wifi_teleop/control/runtime_control_server.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

#include "openarm_wifi_teleop/utils/logging.hpp"

namespace openarm_wifi_teleop {
namespace control {

namespace {

std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

}  // namespace

RuntimeControlServer::RuntimeControlServer(std::string bind_ip, uint16_t port)
    : bind_ip_(std::move(bind_ip)), port_(port), server_fd_(-1), running_(false) {}

RuntimeControlServer::~RuntimeControlServer() {
    stop();
}

void RuntimeControlServer::register_handler(const std::string& command, Handler handler) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    handlers_[command] = std::move(handler);
}

bool RuntimeControlServer::start() {
    if (running_) return true;

    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        LOG_ERROR("RuntimeControlServer socket failed: " << std::strerror(errno));
        return false;
    }

    int reuse = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (inet_pton(AF_INET, bind_ip_.c_str(), &addr.sin_addr) != 1) {
        LOG_ERROR("RuntimeControlServer invalid bind IP: " << bind_ip_);
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    if (bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        LOG_ERROR("RuntimeControlServer bind failed on " << bind_ip_ << ":" << port_
                  << ": " << std::strerror(errno));
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    if (listen(server_fd_, 8) < 0) {
        LOG_ERROR("RuntimeControlServer listen failed: " << std::strerror(errno));
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    running_ = true;
    thread_ = std::thread(&RuntimeControlServer::run, this);
    LOG_INFO("RuntimeControlServer listening on " << bind_ip_ << ":" << port_);
    return true;
}

void RuntimeControlServer::stop() {
    if (!running_ && server_fd_ < 0) return;

    running_ = false;
    if (server_fd_ >= 0) {
        shutdown(server_fd_, SHUT_RDWR);
        close(server_fd_);
        server_fd_ = -1;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

void RuntimeControlServer::run() {
    while (running_) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            if (running_) {
                LOG_WARN("RuntimeControlServer accept failed: " << std::strerror(errno));
            }
            continue;
        }

        char buffer[1024];
        ssize_t n = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        std::string response;
        if (n > 0) {
            buffer[n] = '\0';
            response = handle_command(std::string(buffer));
        } else {
            response = json_error("empty command");
        }
        response.push_back('\n');
        send(client_fd, response.data(), response.size(), MSG_NOSIGNAL);
        close(client_fd);
    }
}

std::string RuntimeControlServer::handle_command(const std::string& line) {
    std::string command = trim(line);
    const auto space = command.find_first_of(" \t");
    std::string args;
    if (space != std::string::npos) {
        args = trim(command.substr(space + 1));
        command = command.substr(0, space);
    }

    Handler handler;
    {
        std::lock_guard<std::mutex> lock(handlers_mutex_);
        auto it = handlers_.find(command);
        if (it == handlers_.end()) {
            return json_error("unknown command: " + command);
        }
        handler = it->second;
    }
    return handler(args);
}

std::string json_escape(const std::string& value) {
    std::ostringstream out;
    for (char ch : value) {
        switch (ch) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << ch; break;
        }
    }
    return out.str();
}

std::string json_ok(const std::string& fields) {
    if (fields.empty()) return "{\"status\":\"ok\"}";
    return "{\"status\":\"ok\"," + fields + "}";
}

std::string json_error(const std::string& message) {
    return "{\"status\":\"error\",\"message\":\"" + json_escape(message) + "\"}";
}

}  // namespace control
}  // namespace openarm_wifi_teleop
