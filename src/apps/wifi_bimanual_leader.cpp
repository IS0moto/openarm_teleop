#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <string>
#include <vector>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <iomanip>

#include "openarm_wifi_teleop/net/udp_sender.hpp"
#include "openarm_wifi_teleop/net/udp_receiver.hpp"
#include "openarm_wifi_teleop/net/feedback_packet.hpp"
#include "openarm_wifi_teleop/core/feedback_state_buffer.hpp"
#include "openarm_wifi_teleop/safety/bilateral_gate.hpp"
#include "openarm_wifi_teleop/utils/link_check.hpp"
#include "openarm_wifi_teleop/net/packet_codec.hpp"
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
std::atomic<bool> command_enabled(false);
std::atomic<bool> init_position_requested(false);
std::atomic<bool> init_position_in_progress(false);
std::atomic<bool> goto_pose_requested(false);
std::atomic<bool> goto_pose_in_progress(false);
std::mutex goto_pose_mutex;
// Target pose: 16 values [R_arm(7), R_grip(1), L_arm(7), L_grip(1)].
std::vector<double> goto_pose_target;

// Bilateral: set by `bilateral_enable` once the link gate passes, cleared by
// `bilateral_disable`, `disable`, estop, or a gate failure while running.
// Phase 1 only requests feedback and monitors the link; no torque from the
// follower is applied to the leader arms yet (that is Phase 2).
std::atomic<bool> bilateral_requested(false);

// Bilateral settings (config/leader.yaml: BilateralFeedback / BilateralLink).
struct BilateralSettings {
    bool feedback_enabled = false;
    std::string bind_ip = "0.0.0.0";
    uint16_t right_port = 50500;
    uint16_t left_port = 50501;
    std::string peer_ip = "";          // follower PC; the physical link is checked toward it
    double eval_window_s = 1.0;
    double link_recheck_s = 1.0;
    openarm_wifi_teleop::safety::BilateralGateConfig gate;
};

static BilateralSettings load_bilateral_settings(const std::string& path) {
    BilateralSettings b;
    try {
        YamlLoader cfg(path);
        if (!cfg.has_node("BilateralFeedback")) return b;
        b.feedback_enabled = cfg.get_bool_or("BilateralFeedback", "Enabled", false);
        b.bind_ip = cfg.get_string_or("BilateralFeedback", "BindIp", b.bind_ip);
        b.right_port = static_cast<uint16_t>(cfg.get_int_or("BilateralFeedback", "RightPort", b.right_port));
        b.left_port = static_cast<uint16_t>(cfg.get_int_or("BilateralFeedback", "LeftPort", b.left_port));
        b.eval_window_s = cfg.get_double_or("BilateralLink", "EvalWindowS", b.eval_window_s);
        b.link_recheck_s = cfg.get_double_or("BilateralLink", "RecheckIntervalS", b.link_recheck_s);
        b.peer_ip = cfg.get_string_or("BilateralLink", "PeerIp", b.peer_ip);
        b.gate.require_wired = cfg.get_bool_or("BilateralLink", "RequireWired", b.gate.require_wired);
        b.gate.allow_loopback = cfg.get_bool_or("BilateralLink", "AllowLoopback", b.gate.allow_loopback);
        b.gate.min_link_speed_mbps = cfg.get_int_or("BilateralLink", "MinLinkSpeedMbps", b.gate.min_link_speed_mbps);
        b.gate.min_feedback_rate_hz = cfg.get_double_or("BilateralLink", "MinFeedbackRateHz", b.gate.min_feedback_rate_hz);
        b.gate.max_feedback_age_ms = cfg.get_double_or("BilateralLink", "MaxFeedbackAgeMs", b.gate.max_feedback_age_ms);
        b.gate.max_rtt_p95_ms = cfg.get_double_or("BilateralLink", "MaxRttP95Ms", b.gate.max_rtt_p95_ms);
        b.gate.max_loss_percent = cfg.get_double_or("BilateralLink", "MaxLossPercent", b.gate.max_loss_percent);
        b.gate.require_follower_link_ok = cfg.get_bool_or("BilateralLink", "RequireFollowerLinkOk", b.gate.require_follower_link_ok);
    } catch (const std::exception& e) {
        LOG_WARN("Bilateral feedback disabled: " << e.what());
        b.feedback_enabled = false;
    }
    return b;
}

static std::string fmt2(double v) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(2) << v;
    return os.str();
}

static std::string feedback_stats_json(const core::FeedbackStateBuffer::Stats& s,
                                       const net::FeedbackPacket* latest) {
    std::string j = "{";
    j += "\"has_data\":" + std::string(s.has_data ? "true" : "false") + ",";
    j += "\"age_ms\":" + fmt2(s.age_ms) + ",";
    j += "\"rate_hz\":" + fmt2(s.rate_hz) + ",";
    j += "\"loss_percent\":" + fmt2(s.loss_percent) + ",";
    j += "\"rtt_p50_ms\":" + fmt2(s.rtt_p50_ms) + ",";
    j += "\"rtt_p95_ms\":" + fmt2(s.rtt_p95_ms) + ",";
    j += "\"rtt_max_ms\":" + fmt2(s.rtt_max_ms) + ",";
    j += "\"cmd_hold_p95_ms\":" + fmt2(s.cmd_hold_p95_ms) + ",";
    j += "\"received_total\":" + std::to_string(s.received_total) + ",";
    j += "\"lost_total\":" + std::to_string(s.lost_total) + ",";
    j += "\"follower_link_ok\":" + std::string((latest && latest->link_ok) ? "true" : "false") + ",";
    j += "\"follower_enabled\":" + std::string((latest && latest->enabled) ? "true" : "false") + ",";
    j += "\"follower_safety_state\":" + std::to_string(latest ? latest->safety_state : 0) + ",";
    j += "\"follower_mode\":" + std::to_string(latest ? latest->mode : 0);
    j += "}";
    return j;
}

// Parse whitespace-separated doubles; return true iff exactly 16 were read.
static bool parse_pose16(const std::string& args, std::vector<double>& out) {
    out.clear();
    std::istringstream iss(args);
    double v;
    while (iss >> v) out.push_back(v);
    return out.size() == 16;
}

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
    bool initial_enable = false;
    bool mock = false;
    std::string bind_ip = "0.0.0.0";
    std::string interface_name = "";
    uint16_t local_port_r = 0;
    uint16_t local_port_l = 0;
    std::string right_urdf = "urdf/openarm_right.urdf";
    std::string left_urdf = "urdf/openarm_left.urdf";
    // Telemetry publishing (identical interface to wifi_bimanual_follower)
    bool publish_telemetry = false;
    std::string telemetry_ip = "127.0.0.1";
    uint16_t telemetry_port = 51000;
    double telemetry_rate_hz = 100.0;
    std::string control_bind_ip = "0.0.0.0";
    uint16_t control_port = 0;

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
        else if (arg == "--enable") initial_enable = true;
        else if (arg == "--mock") mock = true;
        else if (arg == "--publish-telemetry") publish_telemetry = true;
        else if (arg == "--telemetry-ip" && i + 1 < argc) telemetry_ip = argv[++i];
        else if (arg == "--telemetry-port" && i + 1 < argc) telemetry_port = std::stoi(argv[++i]);
        else if (arg == "--telemetry-rate-hz" && i + 1 < argc) telemetry_rate_hz = std::stod(argv[++i]);
        else if (arg == "--control-bind-ip" && i + 1 < argc) control_bind_ip = argv[++i];
        else if (arg == "--control-port" && i + 1 < argc) control_port = std::stoi(argv[++i]);
    }

    command_enabled = initial_enable;

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
        // Optional per-joint gravity trim (defaults: no change). Applied to both arms.
        auto leader_gscale = leader_loader.has("LeaderArmParam", "GravityScale")
                                 ? leader_loader.get_vector("LeaderArmParam", "GravityScale")
                                 : std::vector<double>(leader_kp.size(), 1.0);
        auto leader_goffset = leader_loader.has("LeaderArmParam", "GravityOffset")
                                  ? leader_loader.get_vector("LeaderArmParam", "GravityOffset")
                                  : std::vector<double>(leader_kp.size(), 0.0);
        // Optional per-joint |tau| cap on everything the leader sends (defaults in Control).
        auto leader_effort_limit = leader_loader.has("LeaderArmParam", "EffortLimit")
                                       ? leader_loader.get_vector("LeaderArmParam", "EffortLimit")
                                       : std::vector<double>{};

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
        control_r->SetGravityTrim(leader_gscale, leader_goffset);
        control_r->SetEffortLimits(leader_effort_limit);

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
        control_l->SetGravityTrim(leader_gscale, leader_goffset);
        control_l->SetEffortLimits(leader_effort_limit);

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

    // --- Telemetry publisher ---
    std::unique_ptr<telemetry::OpenArmTelemetryPublisher> telemetry_pub;
    auto telemetry_period = std::chrono::duration<double>(1.0 / telemetry_rate_hz);
    auto next_telemetry_time = std::chrono::steady_clock::now();

    if (publish_telemetry) {
        telemetry_pub = std::make_unique<telemetry::OpenArmTelemetryPublisher>(telemetry_ip, telemetry_port);
        if (!telemetry_pub->start()) {
            LOG_ERROR("Failed to start telemetry publisher");
            return 1;
        }
        LOG_INFO("Telemetry publishing to " << telemetry_ip << ":" << telemetry_port
                 << " at " << telemetry_rate_hz << " Hz");
    }

    // --- Bilateral feedback receive + link gate ---
    BilateralSettings bi = load_bilateral_settings("config/leader.yaml");
    safety::BilateralGate gate(bi.gate);
    core::FeedbackStateBuffer fb_buffer_r(bi.eval_window_s);
    core::FeedbackStateBuffer fb_buffer_l(bi.eval_window_s);
    std::unique_ptr<net::UdpReceiver> fb_receiver_r, fb_receiver_l;
    utils::LinkInfo leader_link;
    safety::BilateralGateResult link_result;
    auto next_link_check = std::chrono::steady_clock::now();
    auto next_gate_check = std::chrono::steady_clock::now();
    std::mutex gate_mutex;  // guards leader_link / link_result (read from the control thread)

    auto refresh_link = [&]() {
        std::lock_guard<std::mutex> lock(gate_mutex);
        if (bi.peer_ip.empty()) {
            leader_link = utils::LinkInfo{};
            leader_link.error = "BilateralLink.PeerIp is not set";
        } else {
            leader_link = utils::query_link_to(bi.peer_ip);
        }
        link_result = gate.check_link(leader_link);
    };

    // Full gate: local link + both arms' feedback quality. Returns reasons on failure.
    auto evaluate_gate = [&]() {
        safety::BilateralGateResult total;
        if (!bi.feedback_enabled) {
            total.reasons.push_back("BilateralFeedback.Enabled is false in config/leader.yaml");
            return total;
        }
        {
            std::lock_guard<std::mutex> lock(gate_mutex);
            total.reasons = link_result.reasons;
        }
        net::FeedbackPacket lr, ll;
        bool hr = fb_buffer_r.get_latest(lr);
        bool hl = fb_buffer_l.get_latest(ll);
        auto rr = gate.check_feedback("right", fb_buffer_r.snapshot(), hr ? &lr : nullptr);
        auto rl = gate.check_feedback("left", fb_buffer_l.snapshot(), hl ? &ll : nullptr);
        total.reasons.insert(total.reasons.end(), rr.reasons.begin(), rr.reasons.end());
        total.reasons.insert(total.reasons.end(), rl.reasons.begin(), rl.reasons.end());
        total.ok = total.reasons.empty();
        return total;
    };

    auto make_feedback_callback = [&](core::FeedbackStateBuffer& buf) {
        return [&buf](const void* data, size_t size) {
            if (size != sizeof(net::FeedbackPacket)) return;
            net::FeedbackPacket fb;
            std::memcpy(&fb, data, sizeof(fb));
            if (!net::PacketCodec::decode_and_validate(fb)) return;
            uint64_t rtt_ns = 0;
            if (fb.echo_cmd_send_time_ns != 0) {
                uint64_t now = utils::system_now_ns();
                if (now > fb.echo_cmd_send_time_ns) rtt_ns = now - fb.echo_cmd_send_time_ns;
            }
            buf.update(fb, rtt_ns);
        };
    };

    if (bi.feedback_enabled) {
        refresh_link();
        LOG_INFO("Bilateral feedback listening on " << bi.bind_ip << ":" << bi.right_port << "/" << bi.left_port
                 << " peer=" << bi.peer_ip << " link=" << leader_link.describe()
                 << " link_ok=" << (link_result.ok ? "true" : "false"));
        if (!link_result.ok) {
            LOG_WARN("Leader link check failed: " << link_result.reasons_joined());
        }
        fb_receiver_r = std::make_unique<net::UdpReceiver>(bi.bind_ip, bi.right_port);
        fb_receiver_l = std::make_unique<net::UdpReceiver>(bi.bind_ip, bi.left_port);
        if (!fb_receiver_r->start_raw(make_feedback_callback(fb_buffer_r)) ||
            !fb_receiver_l->start_raw(make_feedback_callback(fb_buffer_l))) {
            LOG_ERROR("Failed to start bilateral feedback receivers");
            return 1;
        }
    }

    auto bilateral_status_json = [&]() {
        net::FeedbackPacket lr, ll;
        bool hr = fb_buffer_r.get_latest(lr);
        bool hl = fb_buffer_l.get_latest(ll);
        auto g = evaluate_gate();
        std::string link_desc, link_reasons;
        {
            std::lock_guard<std::mutex> lock(gate_mutex);
            link_desc = leader_link.describe();
            link_reasons = link_result.reasons_joined();
        }
        std::string j = "\"bilateral\":{";
        j += "\"feedback_enabled\":" + std::string(bi.feedback_enabled ? "true" : "false") + ",";
        j += "\"requested\":" + std::string(bilateral_requested ? "true" : "false") + ",";
        j += "\"engaged\":false,";  // Phase 2: true when feedback torque is applied to the leader
        j += "\"gate_ok\":" + std::string(g.ok ? "true" : "false") + ",";
        j += "\"gate_reasons\":\"" + control::json_escape(g.reasons_joined()) + "\",";
        j += "\"peer_ip\":\"" + control::json_escape(bi.peer_ip) + "\",";
        j += "\"link\":\"" + control::json_escape(link_desc) + "\",";
        j += "\"link_reasons\":\"" + control::json_escape(link_reasons) + "\",";
        j += "\"right\":" + feedback_stats_json(fb_buffer_r.snapshot(), hr ? &lr : nullptr) + ",";
        j += "\"left\":" + feedback_stats_json(fb_buffer_l.snapshot(), hl ? &ll : nullptr);
        j += "}";
        return j;
    };

    std::unique_ptr<control::RuntimeControlServer> runtime_control;
    if (control_port > 0) {
        runtime_control = std::make_unique<control::RuntimeControlServer>(control_bind_ip, control_port);
        runtime_control->register_handler("bilateral_enable", [&](const std::string&) {
            if (!command_enabled) {
                return control::json_error("leader is not enabled; switch mode to leader first");
            }
            refresh_link();
            auto g = evaluate_gate();
            if (!g.ok) {
                return control::json_error("bilateral refused: " + g.reasons_joined());
            }
            bilateral_requested = true;
            LOG_INFO("Bilateral requested (gate ok).");
            return control::json_ok("\"message\":\"bilateral requested\"," + bilateral_status_json());
        });
        runtime_control->register_handler("bilateral_disable", [&](const std::string&) {
            bilateral_requested = false;
            return control::json_ok("\"message\":\"bilateral disabled\"");
        });
        runtime_control->register_handler("bilateral_status", [&](const std::string&) {
            return control::json_ok(bilateral_status_json());
        });
        runtime_control->register_handler("status", [&](const std::string&) {
            return control::json_ok(
                "\"role\":\"leader\","
                "\"follower_ip\":\"" + control::json_escape(follower_ip) + "\","
                "\"right_port\":" + std::to_string(right_port) + ","
                "\"left_port\":" + std::to_string(left_port) + ","
                "\"enabled\":" + std::string(command_enabled ? "true" : "false") + ","
                "\"init_requested\":" + std::string(init_position_requested ? "true" : "false") + ","
                "\"init_in_progress\":" + std::string(init_position_in_progress ? "true" : "false") + ","
                "\"goto_in_progress\":" + std::string(goto_pose_in_progress ? "true" : "false") + ","
                "\"running\":" + std::string(keep_running ? "true" : "false") + ","
                "\"bilateral_requested\":" + std::string(bilateral_requested ? "true" : "false")
            );
        });
        runtime_control->register_handler("disable", [&](const std::string&) {
            command_enabled = false;
            bilateral_requested = false;
            return control::json_ok("\"message\":\"leader disabled\"");
        });
        runtime_control->register_handler("enable", [&](const std::string&) {
            command_enabled = true;
            return control::json_ok("\"message\":\"leader enabled\"");
        });
        runtime_control->register_handler("init_position", [&](const std::string&) {
            init_position_requested = true;
            return control::json_ok("\"message\":\"leader init_position requested\"");
        });
        runtime_control->register_handler("goto_pose", [&](const std::string& args) {
            std::vector<double> vals;
            if (!parse_pose16(args, vals)) {
                return control::json_error("goto_pose expects 16 whitespace-separated values");
            }
            {
                std::lock_guard<std::mutex> lock(goto_pose_mutex);
                goto_pose_target = vals;
            }
            goto_pose_requested = true;
            return control::json_ok("\"message\":\"leader goto_pose requested\"");
        });
        runtime_control->register_handler("estop", [&](const std::string&) {
            command_enabled = false;
            bilateral_requested = false;
            keep_running = false;
            return control::json_ok("\"message\":\"leader estop requested\"");
        });
        runtime_control->register_handler("shutdown", [&](const std::string&) {
            command_enabled = false;
            keep_running = false;
            return control::json_ok("\"message\":\"leader shutdown requested\"");
        });
        if (!runtime_control->start()) {
            LOG_ERROR("Failed to start leader runtime control server.");
            return 1;
        }
    }

    while (keep_running) {
        if (init_position_requested.exchange(false)) {
            init_position_in_progress = true;
            LOG_INFO("Runtime init_position requested for leader arms.");
            if (!mock && control_r && control_l) {
                std::thread thread_r(&Control::AdjustPosition, control_r);
                std::thread thread_l(&Control::AdjustPosition, control_l);
                thread_r.join();
                thread_l.join();
            }
            init_position_in_progress = false;
            LOG_INFO("Runtime leader init_position complete.");
        }

        if (goto_pose_requested.exchange(false)) {
            goto_pose_in_progress = true;
            std::vector<double> t;
            {
                std::lock_guard<std::mutex> lock(goto_pose_mutex);
                t = goto_pose_target;
            }
            LOG_INFO("Runtime goto_pose requested for leader arms.");
            if (!mock && control_r && control_l && t.size() == 16) {
                std::vector<double> tr(t.begin(), t.begin() + 7), gr{t[7]};
                std::vector<double> tl(t.begin() + 8, t.begin() + 15), gl{t[15]};
                std::thread thread_r([&] { control_r->MoveToPose(tr, gr); });
                std::thread thread_l([&] { control_l->MoveToPose(tl, gl); });
                thread_r.join();
                thread_l.join();
            }
            goto_pose_in_progress = false;
            LOG_INFO("Runtime leader goto_pose complete.");
        }

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

        // Continuous gate while bilateral is requested: link every RecheckIntervalS,
        // feedback quality every 100 ms. Any failure drops back to unilateral.
        if (bilateral_requested) {
            auto tnow = std::chrono::steady_clock::now();
            if (tnow >= next_link_check) {
                refresh_link();
                next_link_check = tnow + std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(bi.link_recheck_s));
            }
            if (tnow >= next_gate_check) {
                auto g = evaluate_gate();
                if (!g.ok) {
                    bilateral_requested = false;
                    LOG_WARN("Bilateral dropped to unilateral: " << g.reasons_joined());
                }
                next_gate_check = tnow + std::chrono::milliseconds(100);
            }
        }
        const uint8_t cmd_mode = static_cast<uint8_t>(
            bilateral_requested ? net::ControlMode::BILATERAL : net::ControlMode::UNILATERAL);

        pkt_r.seq = seq;
        pkt_r.arm_side = static_cast<uint8_t>(net::ArmSide::RIGHT);
        pkt_r.mode = cmd_mode;
        pkt_r.enable = command_enabled ? 1 : 0;
        pkt_r.estop = 0;

        pkt_l.seq = seq;
        pkt_l.arm_side = static_cast<uint8_t>(net::ArmSide::LEFT);
        pkt_l.mode = cmd_mode;
        pkt_l.enable = command_enabled ? 1 : 0;
        pkt_l.estop = 0;
        
        seq++;

        net::PacketCodec::encode(pkt_r);
        net::PacketCodec::encode(pkt_l);
        sender_r.send(pkt_r);
        sender_l.send(pkt_l);

        auto now = std::chrono::steady_clock::now();
        if (now - print_time > std::chrono::seconds(1)) {
            LOG_DEBUG("Sent " << seq << " bimanual packets.");
            if (bi.feedback_enabled) {
                auto sr = fb_buffer_r.snapshot();
                auto sl = fb_buffer_l.snapshot();
                LOG_DEBUG("Feedback [R: " << fmt2(sr.rate_hz) << " Hz rtt p50/p95 " << fmt2(sr.rtt_p50_ms) << "/"
                          << fmt2(sr.rtt_p95_ms) << " ms loss " << fmt2(sr.loss_percent) << "% | L: "
                          << fmt2(sl.rate_hz) << " Hz rtt p50/p95 " << fmt2(sl.rtt_p50_ms) << "/"
                          << fmt2(sl.rtt_p95_ms) << " ms loss " << fmt2(sl.loss_percent) << "%] bilateral="
                          << (bilateral_requested ? "requested" : "off"));
            }
            print_time = now;
        }

        // --- Publish telemetry (leader joint state) ---
        if (publish_telemetry && telemetry_pub && now >= next_telemetry_time) {
            telemetry::OpenArmTelemetryPacketV1 tpkt;
            std::memset(&tpkt, 0, sizeof(tpkt));
            tpkt.monotonic_time_ns = utils::now_ns();
            tpkt.robot_type    = 1;  // openarm_bimanual
            tpkt.control_mode  = 1;  // unilateral_wifi
            tpkt.enabled       = command_enabled ? 1 : 0;
            tpkt.estop         = 0;
            tpkt.state_dim     = 16;
            tpkt.action_dim    = 16;
            tpkt.velocity_dim  = 16;
            tpkt.watchdog_state = 3;  // ACTIVE (leader always active)
            tpkt.safety_state   = 3;

            // Right arm: joints [0..6], gripper [7]
            for (size_t i = 0; i < pkt_r.arm_dof && i < 7; ++i) {
                tpkt.observation_state[i]    = static_cast<float>(pkt_r.arm_pos[i]);
                tpkt.observation_velocity[i] = static_cast<float>(pkt_r.arm_vel[i]);
            }
            if (pkt_r.hand_dof > 0) {
                tpkt.observation_state[7]    = static_cast<float>(pkt_r.hand_pos[0]);
                tpkt.observation_velocity[7] = static_cast<float>(pkt_r.hand_vel[0]);
            }
            // Left arm: joints [8..14], gripper [15]
            for (size_t i = 0; i < pkt_l.arm_dof && i < 7; ++i) {
                tpkt.observation_state[8 + i]    = static_cast<float>(pkt_l.arm_pos[i]);
                tpkt.observation_velocity[8 + i] = static_cast<float>(pkt_l.arm_vel[i]);
            }
            if (pkt_l.hand_dof > 0) {
                tpkt.observation_state[15]    = static_cast<float>(pkt_l.hand_pos[0]);
                tpkt.observation_velocity[15] = static_cast<float>(pkt_l.hand_vel[0]);
            }

            telemetry_pub->publish(tpkt);
            next_telemetry_time = now + std::chrono::duration_cast<std::chrono::nanoseconds>(telemetry_period);
        }

        next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(period);
        std::this_thread::sleep_until(next_time);
    }

    if (fb_receiver_r) fb_receiver_r->stop();
    if (fb_receiver_l) fb_receiver_l->stop();

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
