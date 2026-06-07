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
#include "openarm_wifi_teleop/control/runtime_control_server.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"
#include "openarm_wifi_teleop/utils/network.hpp"
#include "openarm_wifi_teleop/utils/time.hpp"
#include "openarm_wifi_teleop/telemetry/openarm_telemetry_publisher.hpp"
#include "openarm_wifi_teleop/telemetry/openarm_telemetry_packet.hpp"

#include <openarm/can/socket/openarm.hpp>
#include <openarm_port/openarm_init.hpp>
#include <robot_state.hpp>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <yamlloader.hpp>

using namespace openarm_wifi_teleop;

std::atomic<bool> keep_running(true);
std::atomic<bool> init_position_requested(false);
std::atomic<bool> init_position_in_progress(false);
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

const char* safety_state_to_str(safety::SafetyState s) {
    switch(s) {
        case safety::SafetyState::DISABLED: return "DISABLED";
        case safety::SafetyState::WAITING_FOR_ENABLE: return "WAIT_EN";
        case safety::SafetyState::READY: return "READY";
        case safety::SafetyState::ACTIVE: return "ACTIVE";
        case safety::SafetyState::WARNING_TIMEOUT: return "WARN";
        case safety::SafetyState::HOLD: return "HOLD";
        case safety::SafetyState::ESTOP: return "ESTOP";
        case safety::SafetyState::FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

// Verify that motors on a CAN bus are actually responding
bool verify_arm_hardware(openarm::can::socket::OpenArm* arm, const std::string& can_name, const std::string& arm_label) {
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

    std::string right_can = "can2";
    std::string left_can = "can3";
    uint16_t right_port = 50000;
    uint16_t left_port = 50001;
    double rate_hz = 500.0;
    bool mock = false;
    std::string bind_ip = "0.0.0.0";
    std::string interface_name = "";
    std::string right_urdf = "urdf/openarm_right.urdf";
    std::string left_urdf = "urdf/openarm_left.urdf";
    
    bool publish_telemetry = false;
    std::string telemetry_ip = "127.0.0.1";
    uint16_t telemetry_port = 51000;
    double telemetry_rate_hz = 100.0;
    bool telemetry_log_stats = false;
    std::string control_bind_ip = "0.0.0.0";
    uint16_t control_port = 0;
    
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
        else if (arg == "--bind-ip" && i + 1 < argc) bind_ip = argv[++i];
        else if (arg == "--interface" && i + 1 < argc) interface_name = argv[++i];
        else if (arg == "--watchdog-hold-ms" && i + 1 < argc) safety_config.watchdog_hold_ms = std::stod(argv[++i]);
        else if (arg == "--watchdog-disable-ms" && i + 1 < argc) safety_config.watchdog_disable_ms = std::stod(argv[++i]);
        else if (arg == "--publish-telemetry") publish_telemetry = true;
        else if (arg == "--telemetry-ip" && i + 1 < argc) telemetry_ip = argv[++i];
        else if (arg == "--telemetry-port" && i + 1 < argc) telemetry_port = std::stoi(argv[++i]);
        else if (arg == "--telemetry-rate-hz" && i + 1 < argc) telemetry_rate_hz = std::stod(argv[++i]);
        else if (arg == "--telemetry-log-stats") telemetry_log_stats = true;
        else if (arg == "--control-bind-ip" && i + 1 < argc) control_bind_ip = argv[++i];
        else if (arg == "--control-port" && i + 1 < argc) control_port = std::stoi(argv[++i]);
        else if (arg == "--mock") mock = true;
    }

    LOG_INFO("Starting Bimanual Follower");
    LOG_INFO("Mock mode: " << (mock ? "ON" : "OFF"));

    if (!interface_name.empty()) {
        std::string ip = utils::get_interface_ip(interface_name);
        if (!ip.empty()) {
            bind_ip = ip;
        } else {
            LOG_ERROR("Could not find IP for interface: " << interface_name);
            return 1;
        }
    }

    safety_mgr_r = std::make_unique<safety::SafetyManager>(safety_config, "RIGHT");
    safety_mgr_l = std::make_unique<safety::SafetyManager>(safety_config, "LEFT");

    std::unique_ptr<control::RuntimeControlServer> runtime_control;
    if (control_port > 0) {
        runtime_control = std::make_unique<control::RuntimeControlServer>(control_bind_ip, control_port);
        runtime_control->register_handler("status", [&](const std::string&) {
            const auto r_state = safety_mgr_r ? safety_mgr_r->get_state() : safety::SafetyState::DISABLED;
            const auto l_state = safety_mgr_l ? safety_mgr_l->get_state() : safety::SafetyState::DISABLED;
            return control::json_ok(
                "\"role\":\"follower\","
                "\"right_state\":\"" + std::string(safety_state_to_str(r_state)) + "\","
                "\"left_state\":\"" + std::string(safety_state_to_str(l_state)) + "\","
                "\"init_requested\":" + std::string(init_position_requested ? "true" : "false") + ","
                "\"init_in_progress\":" + std::string(init_position_in_progress ? "true" : "false") + ","
                "\"running\":" + std::string(keep_running ? "true" : "false")
            );
        });
        runtime_control->register_handler("init_position", [&](const std::string&) {
            init_position_requested = true;
            return control::json_ok("\"message\":\"follower init_position requested\"");
        });
        runtime_control->register_handler("reset_safety", [&](const std::string&) {
            if (safety_mgr_r) safety_mgr_r->reset();
            if (safety_mgr_l) safety_mgr_l->reset();
            return control::json_ok("\"message\":\"follower safety reset\"");
        });
        runtime_control->register_handler("estop", [&](const std::string&) {
            if (safety_mgr_r) safety_mgr_r->trigger_estop();
            if (safety_mgr_l) safety_mgr_l->trigger_estop();
            return control::json_ok("\"message\":\"follower estop triggered\"");
        });
        runtime_control->register_handler("shutdown", [&](const std::string&) {
            keep_running = false;
            return control::json_ok("\"message\":\"follower shutdown requested\"");
        });
        if (!runtime_control->start()) {
            LOG_ERROR("Failed to start follower runtime control server.");
            return 1;
        }
    }
    
    safety::RateLimiter rate_limiter_r(safety_config.max_target_delta_rad_per_cycle, safety_config.max_joint_velocity_rad_s, 1.0 / rate_hz);
    safety::RateLimiter rate_limiter_l(safety_config.max_target_delta_rad_per_cycle, safety_config.max_joint_velocity_rad_s, 1.0 / rate_hz);
    
    LOG_INFO("Binding to IP: " << bind_ip);
    net::UdpReceiver receiver_r(bind_ip, right_port);
    net::UdpReceiver receiver_l(bind_ip, left_port);
    if (!receiver_r.start(packet_callback_r) || !receiver_l.start(packet_callback_l)) {
        LOG_ERROR("Failed to start UDP receivers.");
        return 1;
    }

    std::unique_ptr<telemetry::OpenArmTelemetryPublisher> telemetry_pub;
    auto telemetry_period = std::chrono::duration<double>(1.0 / telemetry_rate_hz);
    auto next_telemetry_time = std::chrono::steady_clock::now();

    if (publish_telemetry) {
        telemetry_pub = std::make_unique<telemetry::OpenArmTelemetryPublisher>(telemetry_ip, telemetry_port);
        if (!telemetry_pub->start()) {
            LOG_ERROR("Failed to start telemetry publisher");
            return 1;
        }
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
        if (!dynamics_r->Init()) {
            LOG_ERROR("Failed to initialize right arm dynamics");
            return 1;
        }
        follower_arm_r = openarm_init::OpenArmInitializer::initialize_openarm(right_can, true);
        if (!verify_arm_hardware(follower_arm_r, right_can, "RIGHT")) {
            LOG_ERROR("Right follower arm hardware verification failed. Aborting.");
            follower_arm_r->disable_all();
            return 1;
        }
        size_t arm_r_num = follower_arm_r->get_arm().get_motors().size();
        size_t hand_r_num = follower_arm_r->get_gripper().get_motors().size();
        state_r = std::make_shared<RobotSystemState>(arm_r_num, hand_r_num);
        control_r = new Control(follower_arm_r, nullptr, dynamics_r, state_r, 1.0 / rate_hz, ROLE_FOLLOWER, "right_arm", arm_r_num, hand_r_num);
        control_r->SetParameter(follower_kp, follower_kd, follower_Fc, follower_k, follower_Fv, follower_Fo);
        
        // Left
        dynamics_l = new Dynamics(left_urdf, "openarm_body_link0", "openarm_left_hand");
        if (!dynamics_l->Init()) {
            LOG_ERROR("Failed to initialize left arm dynamics");
            return 1;
        }
        follower_arm_l = openarm_init::OpenArmInitializer::initialize_openarm(left_can, true);
        if (!verify_arm_hardware(follower_arm_l, left_can, "LEFT")) {
            LOG_ERROR("Left follower arm hardware verification failed. Aborting.");
            follower_arm_l->disable_all();
            follower_arm_r->disable_all();
            return 1;
        }
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
        if (init_position_requested.exchange(false)) {
            init_position_in_progress = true;
            LOG_INFO("Runtime init_position requested for follower arms.");
            if (!mock && control_r && control_l) {
                std::thread thread_r(&Control::AdjustPosition, control_r);
                std::thread thread_l(&Control::AdjustPosition, control_l);
                thread_r.join();
                thread_l.join();
            }
            if (safety_mgr_r) safety_mgr_r->reset();
            if (safety_mgr_l) safety_mgr_l->reset();
            init_position_in_progress = false;
            LOG_INFO("Runtime follower init_position complete.");
        }

        net::TeleopPacket target_r, target_l;
        bool has_r = state_buffer_r.get_latest(target_r);
        bool has_l = state_buffer_l.get_latest(target_l);
        double gap_r = 0, gap_l = 0;

        if (has_r) {
            uint64_t now_r = utils::now_ns();
            uint64_t last_r = state_buffer_r.get_last_receive_time_ns();
            gap_r = (now_r >= last_r) ? (now_r - last_r) / 1e6 : 0.0;
            safety_mgr_r->update(target_r, gap_r);
        }
        if (has_l) {
            uint64_t now_l = utils::now_ns();
            uint64_t last_l = state_buffer_l.get_last_receive_time_ns();
            gap_l = (now_l >= last_l) ? (now_l - last_l) / 1e6 : 0.0;
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
            LOG_INFO("Status [R: " << safety_state_to_str(state_r_st) << ", Loss: " << state_buffer_r.get_lost_packet_count()
                     << " | L: " << safety_state_to_str(state_l_st) << ", Loss: " << state_buffer_l.get_lost_packet_count() << "]");
            print_time = now;
        }

        if (publish_telemetry && now >= next_telemetry_time) {
            telemetry::OpenArmTelemetryPacketV1 pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.monotonic_time_ns = utils::now_ns();
            pkt.robot_type = 1;
            pkt.control_mode = 1;
            pkt.enabled = (!mock && (state_r_st == safety::SafetyState::ACTIVE || state_l_st == safety::SafetyState::ACTIVE)) ? 1 : 0;
            pkt.estop = (state_r_st == safety::SafetyState::ESTOP || state_l_st == safety::SafetyState::ESTOP) ? 1 : 0;
            pkt.state_dim = 16;
            pkt.action_dim = 16;
            pkt.velocity_dim = 16;
            pkt.watchdog_state = static_cast<uint8_t>(state_r_st);
            pkt.safety_state = static_cast<uint8_t>(state_l_st);

            if (state_r && state_l) {
                auto r_arm_resp = state_r->arm_state().get_all_responses();
                auto r_hand_resp = state_r->hand_state().get_all_responses();
                auto l_arm_resp = state_l->arm_state().get_all_responses();
                auto l_hand_resp = state_l->hand_state().get_all_responses();

                auto r_arm_ref = state_r->arm_state().get_all_references();
                auto r_hand_ref = state_r->hand_state().get_all_references();
                auto l_arm_ref = state_l->arm_state().get_all_references();
                auto l_hand_ref = state_l->hand_state().get_all_references();

                int idx = 0;
                // Right Arm
                for (size_t i = 0; i < r_arm_resp.size() && idx < 16; ++i, ++idx) {
                    pkt.observation_state[idx] = r_arm_resp[i].position;
                    pkt.observation_velocity[idx] = r_arm_resp[i].velocity;
                    pkt.action[idx] = r_arm_ref[i].position;
                }
                // Right Gripper
                for (size_t i = 0; i < r_hand_resp.size() && idx < 16; ++i, ++idx) {
                    pkt.observation_state[idx] = r_hand_resp[i].position;
                    pkt.observation_velocity[idx] = r_hand_resp[i].velocity;
                    pkt.action[idx] = r_hand_ref[i].position;
                }
                // Left Arm
                for (size_t i = 0; i < l_arm_resp.size() && idx < 16; ++i, ++idx) {
                    pkt.observation_state[idx] = l_arm_resp[i].position;
                    pkt.observation_velocity[idx] = l_arm_resp[i].velocity;
                    pkt.action[idx] = l_arm_ref[i].position;
                }
                // Left Gripper
                for (size_t i = 0; i < l_hand_resp.size() && idx < 16; ++i, ++idx) {
                    pkt.observation_state[idx] = l_hand_resp[i].position;
                    pkt.observation_velocity[idx] = l_hand_resp[i].velocity;
                    pkt.action[idx] = l_hand_ref[i].position;
                }
            }

            telemetry_pub->publish(pkt);
            next_telemetry_time += std::chrono::duration_cast<std::chrono::nanoseconds>(telemetry_period);
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
