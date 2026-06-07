#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace openarm_wifi_teleop {
namespace control {

class RuntimeControlServer {
public:
    using Handler = std::function<std::string(const std::string&)>;

    RuntimeControlServer(std::string bind_ip, uint16_t port);
    ~RuntimeControlServer();

    RuntimeControlServer(const RuntimeControlServer&) = delete;
    RuntimeControlServer& operator=(const RuntimeControlServer&) = delete;

    void register_handler(const std::string& command, Handler handler);
    bool start();
    void stop();

private:
    void run();
    std::string handle_command(const std::string& line);

    std::string bind_ip_;
    uint16_t port_;
    int server_fd_;
    std::atomic<bool> running_;
    std::thread thread_;
    std::mutex handlers_mutex_;
    std::map<std::string, Handler> handlers_;
};

std::string json_ok(const std::string& fields = "");
std::string json_error(const std::string& message);
std::string json_escape(const std::string& value);

}  // namespace control
}  // namespace openarm_wifi_teleop
