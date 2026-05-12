#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <string>
#include <vector>
#include <iomanip>
#include <memory>

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
core::TeleopStateBuffer state_buffer_r;
core::TeleopStateBuffer state_buffer_l;
std::unique_ptr<safety::SafetyManager> safety_mgr_r;
std::unique_ptr<safety::SafetyManager> safety_mgr_l;

void signal_handler(int) {
    LOG_INFO("Ctrl+C detected. Triggering ESTOP and shutting down...");
    if (safety_mgr_r) safety_mgr_r->trigger_estop();
    if (safety_mgr_l) safety_mgr_l->trigger_estop();
    keep_running = false;
}

void packet_callback_r(const net::TeleopPacket& packet) { state_buffer_r.update(packet); }
void packet_callback_l(const net::TeleopPacket& packet) { state_buffer_l.update(packet); }

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::string right_can = "can2";
    std::string left_can = "can3";
    uint16_t right_port = 50000;
    uint16_t left_port = 50001;
    double rate_hz = 500.0;
    bool mock = false;
    std::string right_urdf = "";
    std::string left_urdf = "";
    
    safety::SafetyConfig safety_config;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--right-can" && i + 1 < argc) right_can = argv[++i];
        else if (arg == "--left-can" && i + 1 < argc) left_can = argv[++i];
        else if (arg == "--right-port" && i + 1 < argc) right_port = std::stoi(argv[++i]);
        else if (arg == "--left-port" && i + 1 < argc) left_port = std::stoi(argv[++i]);
        else if (arg == "--control-rate-hz" && i + 1 < argc) rate_hz = std::stod(argv[++i]);
        else if (arg == "--right-urdf" && i + 1 < argc) right_urdf = argv[++i];
        else if (arg == "--left-urdf" && i + 1 < argc) left_urdf = argv[++i];
        else if (arg == "--watchdog-hold-ms" && i + 1 < argc) safety_config.watchdog_hold_ms = std::stod(argv[++i]);
        else if (arg == "--watchdog-disable-ms" && i + 1 < argc) safety_config.watchdog_disable_ms = std::stod(argv[++i]);
        else if (arg == "--mock") mock = true;
    }

    LOG_INFO("Starting Bimanual Follower");
    LOG_INFO("Mock mode: " << (mock ? "ON" : "OFF"));

    safety_mgr_r = std::make_unique<safety::SafetyManager>(safety_config);
    safety_mgr_l = std::make_unique<safety::SafetyManager>(safety_config);
    
    safety::RateLimiter rate_limiter_r(safety_config.max_target_delta_rad_per_cycle, safety_config.max_joint_velocity_rad_s, 1.0 / rate_hz);
    safety::RateLimiter rate_limiter_l(safety_config.max_target_delta_rad_per_cycle, safety_config.max_joint_velocity_rad_s, 1.0 / rate_hz);

    net::UdpReceiver receiver_r("0.0.0.0", right_port);
    net::UdpReceiver receiver_l("0.0.0.0", left_port);
    if (!receiver_r.start(packet_callback_r) || !receiver_l.start(packet_callback_l)) {
        LOG_ERROR("Failed to start UDP receivers.");
        return 1;
    }

    openarm::can::socket::OpenArm* follower_arm_r = nullptr;
    openarm::can::socket::OpenArm* follower_arm_l = nullptr;
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

        YamlLoader follower_loader("config/follower.yaml");
        auto follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
        auto follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
        auto follower_Fc = follower_loader.get_vector("FollowerArmParam", "Fc");
        auto follower_k = follower_loader.get_vector("FollowerArmParam", "k");
        auto follower_Fv = follower_loader.get_vector("FollowerArmParam", "Fv");
        auto follower_Fo = follower_loader.get_vector("FollowerArmParam", "Fo");

        // Right
        dynamics_r = new Dynamics(right_urdf, "openarm_body_link0", "openarm_right_hand");
        dynamics_r->Init();
        follower_arm_r = openarm_init::OpenArmInitializer::initialize_openarm(right_can, true);
        size_t arm_r_num = follower_arm_r->get_arm().get_motors().size();
        size_t hand_r_num = follower_arm_r->get_gripper().get_motors().size();
        state_r = std::make_shared<RobotSystemState>(arm_r_num, hand_r_num);
        control_r = new Control(follower_arm_r, nullptr, dynamics_r, state_r, 1.0 / rate_hz, ROLE_FOLLOWER, "right_arm", arm_r_num, hand_r_num);
        control_r->SetParameter(follower_kp, follower_kd, follower_Fc, follower_k, follower_Fv, follower_Fo);
        
        // Left
        dynamics_l = new Dynamics(left_urdf, "openarm_body_link0", "openarm_left_hand");
        dynamics_l->Init();
        follower_arm_l = openarm_init::OpenArmInitializer::initialize_openarm(left_can, true);
        size_t arm_l_num = follower_arm_l->get_arm().get_motors().size();
        size_t hand_l_num = follower_arm_l->get_gripper().get_motors().size();
        state_l = std::make_shared<RobotSystemState>(arm_l_num, hand_l_num);
        control_l = new Control(follower_arm_l, nullptr, dynamics_l, state_l, 1.0 / rate_hz, ROLE_FOLLOWER, "left_arm", arm_l_num, hand_l_num);
        control_l->SetParameter(follower_kp, follower_kd, follower_Fc, follower_k, follower_Fv, follower_Fo);

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
        net::TeleopPacket target_r, target_l;
        bool has_r = state_buffer_r.get_latest(target_r);
        bool has_l = state_buffer_l.get_latest(target_l);
        double gap_r = 0, gap_l = 0;
        uint64_t now_ns = utils::now_ns();

        if (has_r) {
            gap_r = (now_ns - state_buffer_r.get_last_receive_time_ns()) / 1e6;
            safety_mgr_r->update(target_r, gap_r);
        }
        if (has_l) {
            gap_l = (now_ns - state_buffer_l.get_last_receive_time_ns()) / 1e6;
            safety_mgr_l->update(target_l, gap_l);
        }

        safety::SafetyState state_r_st = safety_mgr_r->get_state();
        safety::SafetyState state_l_st = safety_mgr_l->get_state();

        if (!mock) {
            // Right
            if (state_r_st == safety::SafetyState::ACTIVE || state_r_st == safety::SafetyState::READY || state_r_st == safety::SafetyState::WARNING_TIMEOUT) {
                if (has_r) {
                    auto arm_refs = state_r->arm_state().get_all_references();
                    auto hand_refs = state_r->hand_state().get_all_references();
                    std::vector<double> cur_pos, tar_pos;
                    for (const auto& ref : arm_refs) cur_pos.push_back(ref.position);
                    for (const auto& ref : hand_refs) cur_pos.push_back(ref.position);
                    for (int i=0; i<target_r.arm_dof; i++) tar_pos.push_back(target_r.arm_pos[i]);
                    for (int i=0; i<target_r.hand_dof; i++) tar_pos.push_back(target_r.hand_pos[i]);
                    rate_limiter_r.limit(cur_pos, tar_pos);
                    for (size_t i = 0; i < arm_refs.size() && i < tar_pos.size(); ++i) {
                        arm_refs[i].position = tar_pos[i];
                        arm_refs[i].velocity = target_r.arm_vel[i];
                    }
                    size_t arm_off = arm_refs.size();
                    for (size_t i = 0; i < hand_refs.size() && i + arm_off < tar_pos.size(); ++i) {
                        hand_refs[i].position = tar_pos[i + arm_off];
                        hand_refs[i].velocity = target_r.hand_vel[i];
                    }
                    state_r->arm_state().set_all_references(arm_refs);
                    state_r->hand_state().set_all_references(hand_refs);
                }
                control_r->unilateral_step();
            } else if (state_r_st == safety::SafetyState::HOLD) {
                control_r->unilateral_step();
            } else if (state_r_st == safety::SafetyState::ESTOP || state_r_st == safety::SafetyState::FAULT || state_r_st == safety::SafetyState::DISABLED) {
                follower_arm_r->disable_all();
            }

            // Left
            if (state_l_st == safety::SafetyState::ACTIVE || state_l_st == safety::SafetyState::READY || state_l_st == safety::SafetyState::WARNING_TIMEOUT) {
                if (has_l) {
                    auto arm_refs = state_l->arm_state().get_all_references();
                    auto hand_refs = state_l->hand_state().get_all_references();
                    std::vector<double> cur_pos, tar_pos;
                    for (const auto& ref : arm_refs) cur_pos.push_back(ref.position);
                    for (const auto& ref : hand_refs) cur_pos.push_back(ref.position);
                    for (int i=0; i<target_l.arm_dof; i++) tar_pos.push_back(target_l.arm_pos[i]);
                    for (int i=0; i<target_l.hand_dof; i++) tar_pos.push_back(target_l.hand_pos[i]);
                    rate_limiter_l.limit(cur_pos, tar_pos);
                    for (size_t i = 0; i < arm_refs.size() && i < tar_pos.size(); ++i) {
                        arm_refs[i].position = tar_pos[i];
                        arm_refs[i].velocity = target_l.arm_vel[i];
                    }
                    size_t arm_off = arm_refs.size();
                    for (size_t i = 0; i < hand_refs.size() && i + arm_off < tar_pos.size(); ++i) {
                        hand_refs[i].position = tar_pos[i + arm_off];
                        hand_refs[i].velocity = target_l.hand_vel[i];
                    }
                    state_l->arm_state().set_all_references(arm_refs);
                    state_l->hand_state().set_all_references(hand_refs);
                }
                control_l->unilateral_step();
            } else if (state_l_st == safety::SafetyState::HOLD) {
                control_l->unilateral_step();
            } else if (state_l_st == safety::SafetyState::ESTOP || state_l_st == safety::SafetyState::FAULT || state_l_st == safety::SafetyState::DISABLED) {
                follower_arm_l->disable_all();
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (now - print_time > std::chrono::seconds(1)) {
            LOG_INFO("R_Loss: " << state_buffer_r.get_lost_packet_count() << " | L_Loss: " << state_buffer_l.get_lost_packet_count());
            print_time = now;
        }

        next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(period);
        std::this_thread::sleep_until(next_time);
    }

    receiver_r.stop();
    receiver_l.stop();
    if (!mock) {
        if (follower_arm_r) follower_arm_r->disable_all();
        if (follower_arm_l) follower_arm_l->disable_all();
    }

    delete control_r;
    delete control_l;
    delete dynamics_r;
    delete dynamics_l;

    return 0;
}
