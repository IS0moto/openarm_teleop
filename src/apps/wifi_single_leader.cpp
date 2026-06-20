#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <string>
#include <vector>
#include <cmath>
#include <cstring>

#include "openarm_wifi_teleop/net/udp_sender.hpp"
#include "openarm_wifi_teleop/net/packet_codec.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"
#include "openarm_wifi_teleop/utils/time.hpp"

#include <openarm/can/socket/openarm.hpp>
#include <openarm_port/openarm_init.hpp>
#include <robot_state.hpp>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <yamlloader.hpp>

using namespace openarm_wifi_teleop;

std::atomic<bool> keep_running(true);

void signal_handler(int) {
    LOG_INFO("Ctrl+C detected. Shutting down...");
    keep_running = false;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::string follower_ip = "127.0.0.1";
    std::string can_interface = "can0";
    uint16_t port = 50000;
    double rate_hz = 500.0;
    bool enable = false;
    bool mock = false;
    std::string leader_urdf = "";
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--follower-ip" && i + 1 < argc) follower_ip = argv[++i];
        else if (arg == "--can" && i + 1 < argc) can_interface = argv[++i];
        else if (arg == "--port" && i + 1 < argc) port = std::stoi(argv[++i]);
        else if (arg == "--rate-hz" && i + 1 < argc) rate_hz = std::stod(argv[++i]);
        else if (arg == "--urdf" && i + 1 < argc) leader_urdf = argv[++i];
        else if (arg == "--enable") enable = true;
        else if (arg == "--mock") mock = true;
    }

    LOG_INFO("Starting Single Leader (Right Arm)");
    LOG_INFO("Follower IP: " << follower_ip << ":" << port);
    LOG_INFO("Rate: " << rate_hz << " Hz");
    LOG_INFO("Mock mode: " << (mock ? "ON" : "OFF"));
    LOG_INFO("Enable signal: " << (enable ? "ON" : "OFF"));

    net::UdpSender sender(follower_ip, port);
    uint32_t seq = 0;
    
    openarm::can::socket::OpenArm* leader_openarm = nullptr;
    Dynamics* dynamics_l = nullptr;
    Control* control_leader = nullptr;
    std::shared_ptr<RobotSystemState> leader_state;
    
    if (!mock) {
        if (leader_urdf.empty()) {
            LOG_ERROR("--urdf is required in real mode");
            return 1;
        }

        YamlLoader leader_loader("config/leader.yaml");
        std::vector<double> leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
        std::vector<double> leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
        std::vector<double> leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
        std::vector<double> leader_k = leader_loader.get_vector("LeaderArmParam", "k");
        std::vector<double> leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
        std::vector<double> leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");

        dynamics_l = new Dynamics(leader_urdf, "openarm_body_link0", "openarm_right_hand");
        dynamics_l->Init();

        leader_openarm = openarm_init::OpenArmInitializer::initialize_openarm(can_interface, true);
        size_t arm_motor_num = leader_openarm->get_arm().get_motors().size();
        size_t hand_motor_num = leader_openarm->get_gripper().get_motors().size();

        leader_state = std::make_shared<RobotSystemState>(arm_motor_num, hand_motor_num);

        control_leader = new Control(
            leader_openarm, dynamics_l, nullptr, leader_state,
            1.0 / rate_hz, ROLE_LEADER, "right_arm", arm_motor_num, hand_motor_num);

        control_leader->SetParameter(leader_kp, leader_kd, leader_Fc, leader_k, leader_Fv, leader_Fo);
        
        LOG_INFO("Adjusting position...");
        control_leader->AdjustPosition();
        LOG_INFO("Adjust complete.");
    }

    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    auto next_time = std::chrono::steady_clock::now();

    auto print_time = std::chrono::steady_clock::now();

    while (keep_running) {
        net::TeleopPacket packet;
        std::memset(&packet, 0, sizeof(packet));

        if (!mock) {
            control_leader->unilateral_step();

            auto arm_responses = leader_state->arm_state().get_all_responses();
            auto hand_responses = leader_state->hand_state().get_all_responses();

            packet.arm_dof = arm_responses.size();
            packet.hand_dof = hand_responses.size();

            for (size_t i = 0; i < packet.arm_dof && i < 8; ++i) {
                packet.arm_pos[i] = arm_responses[i].position;
                packet.arm_vel[i] = arm_responses[i].velocity;
            }
            for (size_t i = 0; i < packet.hand_dof && i < 4; ++i) {
                packet.hand_pos[i] = hand_responses[i].position;
                packet.hand_vel[i] = hand_responses[i].velocity;
            }
        } else {
            double t = (utils::system_now_ns() / 1e9);
            packet.arm_dof = 7;
            packet.hand_dof = 1;
            for (int i = 0; i < 7; ++i) {
                packet.arm_pos[i] = std::sin(t * 2.0 + i);
                packet.arm_vel[i] = std::cos(t * 2.0 + i) * 2.0;
            }
            packet.hand_pos[0] = 0.5 * (1.0 + std::sin(t));
        }

        packet.seq = seq++;
        packet.arm_side = static_cast<uint8_t>(net::ArmSide::RIGHT);
        packet.mode = static_cast<uint8_t>(net::ControlMode::UNILATERAL);
        packet.enable = enable ? 1 : 0;
        packet.estop = 0;

        net::PacketCodec::encode(packet);
        sender.send(packet);

        auto now = std::chrono::steady_clock::now();
        if (now - print_time > std::chrono::seconds(1)) {
            LOG_DEBUG("Sent " << sender.get_sent_count() << " packets.");
            print_time = now;
        }

        next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(period);
        std::this_thread::sleep_until(next_time);
    }

    if (!mock && leader_openarm) {
        leader_openarm->disable_all();
    }

    delete control_leader;
    delete dynamics_l;

    return 0;
}
