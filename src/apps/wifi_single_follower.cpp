#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <string>
#include <vector>
#include <iomanip>

#include "openarm_wifi_teleop/net/udp_receiver.hpp"
#include "openarm_wifi_teleop/core/teleop_state_buffer.hpp"
#include "openarm_wifi_teleop/safety/safety_manager.hpp"
#include "openarm_wifi_teleop/safety/rate_limiter.hpp"
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
core::TeleopStateBuffer state_buffer;
std::unique_ptr<safety::SafetyManager> safety_mgr;

void signal_handler(int) {
    LOG_INFO("Ctrl+C detected. Triggering ESTOP and shutting down...");
    if (safety_mgr) {
        safety_mgr->trigger_estop();
    }
    keep_running = false;
}

void packet_callback(const net::TeleopPacket& packet) {
    state_buffer.update(packet);
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::string can_interface = "can2";
    uint16_t port = 50000;
    double rate_hz = 500.0;
    bool mock = false;
    std::string follower_urdf = "";
    
    safety::SafetyConfig safety_config;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--can" && i + 1 < argc) can_interface = argv[++i];
        else if (arg == "--port" && i + 1 < argc) port = std::stoi(argv[++i]);
        else if (arg == "--control-rate-hz" && i + 1 < argc) rate_hz = std::stod(argv[++i]);
        else if (arg == "--urdf" && i + 1 < argc) follower_urdf = argv[++i];
        else if (arg == "--watchdog-hold-ms" && i + 1 < argc) safety_config.watchdog_hold_ms = std::stod(argv[++i]);
        else if (arg == "--watchdog-disable-ms" && i + 1 < argc) safety_config.watchdog_disable_ms = std::stod(argv[++i]);
        else if (arg == "--mock") mock = true;
    }

    LOG_INFO("Starting Single Follower (Right Arm)");
    LOG_INFO("Port: " << port);
    LOG_INFO("Rate: " << rate_hz << " Hz");
    LOG_INFO("Mock mode: " << (mock ? "ON" : "OFF"));

    safety_mgr = std::make_unique<safety::SafetyManager>(safety_config);
    safety::RateLimiter rate_limiter(safety_config.max_target_delta_rad_per_cycle, safety_config.max_joint_velocity_rad_s, 1.0 / rate_hz);

    net::UdpReceiver receiver("0.0.0.0", port);
    if (!receiver.start(packet_callback)) {
        LOG_ERROR("Failed to start UDP receiver.");
        return 1;
    }

    openarm::can::socket::OpenArm* follower_openarm = nullptr;
    Dynamics* dynamics_f = nullptr;
    Control* control_follower = nullptr;
    std::shared_ptr<RobotSystemState> follower_state;
    
    if (!mock) {
        if (follower_urdf.empty()) {
            LOG_ERROR("--urdf is required in real mode");
            return 1;
        }

        YamlLoader follower_loader("config/follower.yaml");
        std::vector<double> follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
        std::vector<double> follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
        std::vector<double> follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
        std::vector<double> follower_k = follower_loader.get_vector("FollowerArmParam", "k");
        std::vector<double> follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
        std::vector<double> follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");

        dynamics_f = new Dynamics(follower_urdf, "openarm_body_link0", "openarm_right_hand");
        dynamics_f->Init();

        follower_openarm = openarm_init::OpenArmInitializer::initialize_openarm(can_interface, true);
        size_t arm_motor_num = follower_openarm->get_arm().get_motors().size();
        size_t hand_motor_num = follower_openarm->get_gripper().get_motors().size();

        follower_state = std::make_shared<RobotSystemState>(arm_motor_num, hand_motor_num);

        control_follower = new Control(
            follower_openarm, nullptr, dynamics_f, follower_state,
            1.0 / rate_hz, ROLE_FOLLOWER, "right_arm", arm_motor_num, hand_motor_num);

        control_follower->SetParameter(follower_kp, follower_kd, follower_Fc, follower_k, follower_Fv, follower_Fo);
        
        LOG_INFO("Adjusting position...");
        control_follower->AdjustPosition();
        LOG_INFO("Adjust complete.");
    }

    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    auto next_time = std::chrono::steady_clock::now();
    auto print_time = std::chrono::steady_clock::now();

    while (keep_running) {
        net::TeleopPacket target_packet;
        bool has_target = state_buffer.get_latest(target_packet);
        double gap_ms = 0;

        if (has_target) {
            uint64_t now_ns = utils::now_ns();
            uint64_t last_rx = state_buffer.get_last_receive_time_ns();
            gap_ms = (now_ns - last_rx) / 1e6;
            
            safety_mgr->update(target_packet, gap_ms);
        }

        safety::SafetyState current_state = safety_mgr->get_state();

        if (!mock) {
            if (current_state == safety::SafetyState::ACTIVE || current_state == safety::SafetyState::READY || current_state == safety::SafetyState::WARNING_TIMEOUT) {
                if (has_target && follower_state) {
                    auto arm_refs = follower_state->arm_state().get_all_references();
                    auto hand_refs = follower_state->hand_state().get_all_references();
                    
                    std::vector<double> current_pos;
                    for (const auto& ref : arm_refs) current_pos.push_back(ref.position);
                    for (const auto& ref : hand_refs) current_pos.push_back(ref.position);
                    
                    std::vector<double> target_pos;
                    for (int i=0; i<target_packet.arm_dof; i++) target_pos.push_back(target_packet.arm_pos[i]);
                    for (int i=0; i<target_packet.hand_dof; i++) target_pos.push_back(target_packet.hand_pos[i]);
                    
                    rate_limiter.limit(current_pos, target_pos);

                    for (size_t i = 0; i < arm_refs.size() && i < target_pos.size(); ++i) {
                        arm_refs[i].position = target_pos[i];
                        arm_refs[i].velocity = target_packet.arm_vel[i];
                    }
                    size_t arm_offset = arm_refs.size();
                    for (size_t i = 0; i < hand_refs.size() && i + arm_offset < target_pos.size(); ++i) {
                        hand_refs[i].position = target_pos[i + arm_offset];
                        hand_refs[i].velocity = target_packet.hand_vel[i];
                    }

                    follower_state->arm_state().set_all_references(arm_refs);
                    follower_state->hand_state().set_all_references(hand_refs);
                }

                control_follower->unilateral_step();
            } else if (current_state == safety::SafetyState::HOLD) {
                control_follower->unilateral_step();
            } else if (current_state == safety::SafetyState::ESTOP || current_state == safety::SafetyState::FAULT || current_state == safety::SafetyState::DISABLED) {
                follower_openarm->disable_all();
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (now - print_time > std::chrono::seconds(1)) {
            std::string state_str = "UNKNOWN";
            switch(current_state) {
                case safety::SafetyState::DISABLED: state_str = "DISABLED"; break;
                case safety::SafetyState::WAITING_FOR_ENABLE: state_str = "WAIT_ENABLE"; break;
                case safety::SafetyState::READY: state_str = "READY"; break;
                case safety::SafetyState::ACTIVE: state_str = "ACTIVE"; break;
                case safety::SafetyState::WARNING_TIMEOUT: state_str = "WARNING"; break;
                case safety::SafetyState::HOLD: state_str = "HOLD"; break;
                case safety::SafetyState::ESTOP: state_str = "ESTOP"; break;
                case safety::SafetyState::FAULT: state_str = "FAULT"; break;
            }
            
            LOG_INFO("State: " << state_str 
                     << " | RX: " << receiver.get_received_count() 
                     << " | Loss: " << state_buffer.get_lost_packet_count()
                     << " | Gap: " << std::fixed << std::setprecision(1) << gap_ms << "ms"
                     << " | Period: " << state_buffer.get_average_receive_period_ms() << "ms");
            print_time = now;
        }

        next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(period);
        std::this_thread::sleep_until(next_time);
    }

    receiver.stop();
    if (!mock && follower_openarm) {
        follower_openarm->disable_all();
    }

    delete control_follower;
    delete dynamics_f;

    return 0;
}
