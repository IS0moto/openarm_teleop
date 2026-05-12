#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <string>
#include <vector>
#include <cstring>
#include <memory>

#include "openarm_wifi_teleop/net/udp_sender.hpp"
#include "openarm_wifi_teleop/net/packet_codec.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"
#include "openarm_wifi_teleop/utils/network.hpp"
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

// Verify that motors on a CAN bus are actually responding
bool verify_arm_hardware(openarm::can::socket::OpenArm* arm, const std::string& can_name, const std::string& arm_label) {
    // Re-read motor states to ensure we have fresh data
    arm->recv_all(1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    arm->recv_all(1000);

    auto arm_motors = arm->get_arm().get_motors();
    auto grip_motors = arm->get_gripper().get_motors();

    int arm_responding = 0, arm_total = arm_motors.size();
    int grip_responding = 0, grip_total = grip_motors.size();

    LOG_INFO("[" << arm_label << "] Verifying " << arm_total << " arm motors + " << grip_total << " gripper motors on " << can_name);

    for (size_t i = 0; i < arm_motors.size(); ++i) {
        const auto& m = arm_motors[i];
        bool has_data = (m.get_position() != 0.0 || m.get_velocity() != 0.0);
        if (has_data) arm_responding++;
        LOG_INFO("  [" << arm_label << "] Arm motor " << i
                 << ": responding=" << (has_data ? "YES" : "NO")
                 << " pos=" << m.get_position()
                 << " vel=" << m.get_velocity());
    }
    for (size_t i = 0; i < grip_motors.size(); ++i) {
        const auto& m = grip_motors[i];
        bool has_data = (m.get_position() != 0.0 || m.get_velocity() != 0.0);
        if (has_data) grip_responding++;
        LOG_INFO("  [" << arm_label << "] Gripper motor " << i
                 << ": responding=" << (has_data ? "YES" : "NO")
                 << " pos=" << m.get_position()
                 << " vel=" << m.get_velocity());
    }

    if (arm_responding == 0 && grip_responding == 0) {
        LOG_ERROR("[" << arm_label << "] NO motors responding on " << can_name << "! Check CAN bus and hardware power.");
        return false;
    }
    if (arm_responding < arm_total || grip_responding < grip_total) {
        LOG_WARN("[" << arm_label << "] Only " << arm_responding << "/" << arm_total
                 << " arm motors and " << grip_responding << "/" << grip_total
                 << " gripper motors responding on " << can_name);
    } else {
        LOG_INFO("[" << arm_label << "] All motors verified OK on " << can_name);
    }
    return true;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::string follower_ip = "172.30.21.146";
    std::string right_can = "can0";
    std::string left_can = "can1";
    uint16_t right_port = 50000;
    uint16_t left_port = 50001;
    double rate_hz = 500.0;
    bool enable = false;
    bool mock = false;
    std::string bind_ip = "0.0.0.0";
    std::string interface_name = "";
    uint16_t local_port_r = 0;
    uint16_t local_port_l = 0;
    std::string right_urdf = "urdf/openarm_right.urdf";
    std::string left_urdf = "urdf/openarm_left.urdf";
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--follower-ip" && i + 1 < argc) follower_ip = argv[++i];
        else if (arg == "--right-can" && i + 1 < argc) right_can = argv[++i];
        else if (arg == "--left-can" && i + 1 < argc) left_can = argv[++i];
        else if (arg == "--right-port" && i + 1 < argc) right_port = std::stoi(argv[++i]);
        else if (arg == "--left-port" && i + 1 < argc) left_port = std::stoi(argv[++i]);
        else if (arg == "--rate-hz" && i + 1 < argc) rate_hz = std::stod(argv[++i]);
        else if (arg == "--right-urdf" && i + 1 < argc) right_urdf = argv[++i];
        else if (arg == "--left-urdf" && i + 1 < argc) left_urdf = argv[++i];
        else if (arg == "--bind-ip" && i + 1 < argc) bind_ip = argv[++i];
        else if (arg == "--interface" && i + 1 < argc) interface_name = argv[++i];
        else if (arg == "--local-port-r" && i + 1 < argc) local_port_r = std::stoi(argv[++i]);
        else if (arg == "--local-port-l" && i + 1 < argc) local_port_l = std::stoi(argv[++i]);
        else if (arg == "--enable") enable = true;
        else if (arg == "--mock") mock = true;
    }

    if (!interface_name.empty()) {
        std::string ip = utils::get_interface_ip(interface_name);
        if (!ip.empty()) {
            bind_ip = ip;
        } else {
            LOG_ERROR("Could not find IP for interface: " << interface_name);
            return 1;
        }
    }

    LOG_INFO("Starting Bimanual Leader");
    LOG_INFO("Follower IP: " << follower_ip << " (" << right_port << ", " << left_port << ")");
    LOG_INFO("Binding to: " << bind_ip << " (Ports: " << local_port_r << ", " << local_port_l << ")");
    LOG_INFO("Mock mode: " << (mock ? "ON" : "OFF"));

    net::UdpSender sender_r(follower_ip, right_port, bind_ip, local_port_r);
    net::UdpSender sender_l(follower_ip, left_port, bind_ip, local_port_l);
    uint32_t seq = 0;
    
    openarm::can::socket::OpenArm* leader_arm_r = nullptr;
    openarm::can::socket::OpenArm* leader_arm_l = nullptr;
    Dynamics* dynamics_r = nullptr;
    Dynamics* dynamics_l = nullptr;
    Control* control_r = nullptr;
    Control* control_l = nullptr;
    std::shared_ptr<RobotSystemState> state_r;
    std::shared_ptr<RobotSystemState> state_l;
    
    if (!mock) {
        if (right_urdf.empty() || left_urdf.empty()) {
            LOG_ERROR("--right-urdf and --left-urdf are required in real mode");
            return 1;
        }

        YamlLoader leader_loader("config/leader.yaml");
        auto leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
        auto leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
        auto leader_Fc = leader_loader.get_vector("LeaderArmParam", "Fc");
        auto leader_k = leader_loader.get_vector("LeaderArmParam", "k");
        auto leader_Fv = leader_loader.get_vector("LeaderArmParam", "Fv");
        auto leader_Fo = leader_loader.get_vector("LeaderArmParam", "Fo");

        dynamics_r = new Dynamics(right_urdf, "openarm_body_link0", "openarm_right_hand");
        if (!dynamics_r->Init()) {
            LOG_ERROR("Failed to initialize right arm dynamics");
            return 1;
        }
        leader_arm_r = openarm_init::OpenArmInitializer::initialize_openarm(right_can, true);
        if (!leader_arm_r) {
            LOG_ERROR("Failed to initialize right arm on " << right_can);
            return 1;
        }
        if (!verify_arm_hardware(leader_arm_r, right_can, "RIGHT")) {
            LOG_ERROR("Right arm hardware verification failed. Aborting.");
            leader_arm_r->disable_all();
            return 1;
        }
        size_t arm_r_num = leader_arm_r->get_arm().get_motors().size();
        size_t hand_r_num = leader_arm_r->get_gripper().get_motors().size();
        state_r = std::make_shared<RobotSystemState>(arm_r_num, hand_r_num);
        control_r = new Control(leader_arm_r, dynamics_r, nullptr, state_r, 1.0 / rate_hz, ROLE_LEADER, "right_arm", arm_r_num, hand_r_num);
        control_r->SetParameter(leader_kp, leader_kd, leader_Fc, leader_k, leader_Fv, leader_Fo);
        
        dynamics_l = new Dynamics(left_urdf, "openarm_body_link0", "openarm_left_hand");
        if (!dynamics_l->Init()) {
            LOG_ERROR("Failed to initialize left arm dynamics");
            return 1;
        }
        leader_arm_l = openarm_init::OpenArmInitializer::initialize_openarm(left_can, true);
        if (!leader_arm_l) {
            LOG_ERROR("Failed to initialize left arm on " << left_can);
            return 1;
        }
        if (!verify_arm_hardware(leader_arm_l, left_can, "LEFT")) {
            LOG_ERROR("Left arm hardware verification failed. Aborting.");
            leader_arm_l->disable_all();
            leader_arm_r->disable_all();
            return 1;
        }
        size_t arm_l_num = leader_arm_l->get_arm().get_motors().size();
        size_t hand_l_num = leader_arm_l->get_gripper().get_motors().size();
        state_l = std::make_shared<RobotSystemState>(arm_l_num, hand_l_num);
        control_l = new Control(leader_arm_l, dynamics_l, nullptr, state_l, 1.0 / rate_hz, ROLE_LEADER, "left_arm", arm_l_num, hand_l_num);
        control_l->SetParameter(leader_kp, leader_kd, leader_Fc, leader_k, leader_Fv, leader_Fo);

        LOG_INFO("Adjusting position...");
        std::thread thread_r(&Control::AdjustPosition, control_r);
        std::thread thread_l(&Control::AdjustPosition, control_l);
        thread_r.join();
        thread_l.join();
        LOG_INFO("Adjust complete.");
    }

    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    auto next_time = std::chrono::steady_clock::now();
    auto print_time = std::chrono::steady_clock::now();

    while (keep_running) {
        net::TeleopPacket pkt_r, pkt_l;
        std::memset(&pkt_r, 0, sizeof(pkt_r));
        std::memset(&pkt_l, 0, sizeof(pkt_l));

        if (!mock) {
            control_r->unilateral_step();
            control_l->unilateral_step();

            auto arm_res_r = state_r->arm_state().get_all_responses();
            auto hand_res_r = state_r->hand_state().get_all_responses();
            pkt_r.arm_dof = arm_res_r.size();
            pkt_r.hand_dof = hand_res_r.size();
            for (size_t i = 0; i < pkt_r.arm_dof && i < 8; ++i) {
                pkt_r.arm_pos[i] = arm_res_r[i].position;
                pkt_r.arm_vel[i] = arm_res_r[i].velocity;
            }
            for (size_t i = 0; i < pkt_r.hand_dof && i < 4; ++i) {
                pkt_r.hand_pos[i] = hand_res_r[i].position;
                pkt_r.hand_vel[i] = hand_res_r[i].velocity;
            }

            auto arm_res_l = state_l->arm_state().get_all_responses();
            auto hand_res_l = state_l->hand_state().get_all_responses();
            pkt_l.arm_dof = arm_res_l.size();
            pkt_l.hand_dof = hand_res_l.size();
            for (size_t i = 0; i < pkt_l.arm_dof && i < 8; ++i) {
                pkt_l.arm_pos[i] = arm_res_l[i].position;
                pkt_l.arm_vel[i] = arm_res_l[i].velocity;
            }
            for (size_t i = 0; i < pkt_l.hand_dof && i < 4; ++i) {
                pkt_l.hand_pos[i] = hand_res_l[i].position;
                pkt_l.hand_vel[i] = hand_res_l[i].velocity;
            }
        } else {
            double t = (utils::system_now_ns() / 1e9);
            pkt_r.arm_dof = 7; pkt_r.hand_dof = 1;
            pkt_l.arm_dof = 7; pkt_l.hand_dof = 1;
            for (int i = 0; i < 7; ++i) {
                pkt_r.arm_pos[i] = std::sin(t * 2.0 + i);
                pkt_l.arm_pos[i] = std::cos(t * 2.0 + i);
            }
        }

        pkt_r.seq = seq;
        pkt_r.arm_side = static_cast<uint8_t>(net::ArmSide::RIGHT);
        pkt_r.mode = static_cast<uint8_t>(net::ControlMode::UNILATERAL);
        pkt_r.enable = enable ? 1 : 0;
        pkt_r.estop = 0;

        pkt_l.seq = seq;
        pkt_l.arm_side = static_cast<uint8_t>(net::ArmSide::LEFT);
        pkt_l.mode = static_cast<uint8_t>(net::ControlMode::UNILATERAL);
        pkt_l.enable = enable ? 1 : 0;
        pkt_l.estop = 0;
        
        seq++;

        net::PacketCodec::encode(pkt_r);
        net::PacketCodec::encode(pkt_l);
        sender_r.send(pkt_r);
        sender_l.send(pkt_l);

        auto now = std::chrono::steady_clock::now();
        if (now - print_time > std::chrono::seconds(1)) {
            LOG_INFO("Sent " << seq << " bimanual packets.");
            print_time = now;
        }

        next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(period);
        std::this_thread::sleep_until(next_time);
    }

    if (!mock) {
        if (leader_arm_r) leader_arm_r->disable_all();
        if (leader_arm_l) leader_arm_l->disable_all();
    }

    delete control_r;
    delete control_l;
    delete dynamics_r;
    delete dynamics_l;

    return 0;
}
