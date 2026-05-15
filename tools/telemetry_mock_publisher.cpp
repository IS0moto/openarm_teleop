#include "openarm_wifi_teleop/telemetry/openarm_telemetry_publisher.hpp"
#include "openarm_wifi_teleop/utils/time.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>
#include <cstring>

using namespace openarm_wifi_teleop;
using namespace openarm_wifi_teleop::telemetry;

std::atomic<bool> keep_running(true);

void signal_handler(int) {
    keep_running = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::string ip = "127.0.0.1";
    uint16_t port = 51000;
    double rate_hz = 100.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--telemetry-ip" && i + 1 < argc) ip = argv[++i];
        else if (arg == "--telemetry-port" && i + 1 < argc) port = std::stoi(argv[++i]);
        else if (arg == "--rate-hz" && i + 1 < argc) rate_hz = std::stod(argv[++i]);
    }

    OpenArmTelemetryPublisher pub(ip, port);
    if (!pub.start()) {
        std::cerr << "Failed to start publisher\n";
        return 1;
    }

    std::cout << "Mock publisher running at " << rate_hz << " Hz...\n";

    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    auto next_time = std::chrono::steady_clock::now();
    
    double t = 0.0;

    while (keep_running) {
        OpenArmTelemetryPacketV1 pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.monotonic_time_ns = utils::now_ns();
        pkt.robot_type = 1;
        pkt.control_mode = 1;
        pkt.enabled = 1;
        pkt.estop = 0;
        pkt.state_dim = 16;
        pkt.action_dim = 16;
        pkt.velocity_dim = 16;
        pkt.watchdog_state = 3; // ACTIVE
        pkt.safety_state = 3;

        for (int i = 0; i < 16; ++i) {
            pkt.observation_state[i] = std::sin(t + i * 0.1);
            pkt.observation_velocity[i] = std::cos(t + i * 0.1);
            pkt.action[i] = std::sin(t + i * 0.1 + 0.05); // Action is slightly ahead
        }

        pub.publish(pkt);

        t += 1.0 / rate_hz;
        next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(period);
        std::this_thread::sleep_until(next_time);
    }

    pub.stop();
    return 0;
}
