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
#include <iomanip>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>

#include "openarm_wifi_teleop/net/udp_sender.hpp"
#include "openarm_wifi_teleop/net/udp_receiver.hpp"
#include "openarm_wifi_teleop/net/vr_protocol.hpp"
#include "openarm_wifi_teleop/net/packet_codec.hpp"
#include "openarm_wifi_teleop/utils/logging.hpp"
#include "openarm_wifi_teleop/utils/network.hpp"
#include "openarm_wifi_teleop/utils/time.hpp"
#include "openarm_wifi_teleop/ik/openarm_pinocchio_ik.hpp"
#include "openarm_wifi_teleop/telemetry/openarm_telemetry_packet.hpp"
#include "openarm_wifi_teleop/control/temporal_swivel_estimator.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

using namespace openarm_wifi_teleop;
using namespace openarm_teleop;

std::atomic<bool> keep_running(true);
void signal_handler(int) { keep_running = false; }

static std::string vec_to_string(const Eigen::Vector3d& v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
    return oss.str();
}

static void log_mapping_basis(const std::string& label, const Eigen::Matrix3d& mat) {
    const Eigen::Vector3d openxr_right = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d openxr_up = Eigen::Vector3d::UnitY();
    const Eigen::Vector3d openxr_forward = -Eigen::Vector3d::UnitZ();
    const double det = mat.determinant();

    LOG_INFO(label << " mapping basis:"
             << " OpenXR +X/right -> robot " << vec_to_string(mat * openxr_right)
             << ", +Y/up -> robot " << vec_to_string(mat * openxr_up)
             << ", -Z/forward -> robot " << vec_to_string(mat * openxr_forward)
             << ", det=" << std::fixed << std::setprecision(3) << det);

    if (std::abs(det - 1.0) > 1e-6) {
        LOG_WARN(label << " mapping determinant is " << det
                 << "; expected +1 for a proper right-handed axis transform.");
    }
}

struct VrTeleopConfig {
    struct VrMapping {
        std::array<double, 3> position_scale_xyz = {1,1,1};
        double max_delta_m = 0.30;
        double deadband_m = 0.002;
        double max_delta_per_cycle_m = 0.01;
        Eigen::Matrix3d right_translation_matrix = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d left_translation_matrix = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d right_rotation_matrix = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d left_rotation_matrix = Eigen::Matrix3d::Identity();
        bool invert_rotation_delta = false;
        std::string rotation_compose_order = "anchor_then_delta"; // or "delta_then_anchor"
    } mapping;

    struct IKConfig {
        std::string mode = "pose";
        int max_iterations = 20;
        double convergence_tol_m = 0.005;
        double convergence_tol_rad = 0.05;
        double damping = 0.05;
        double max_dq_norm = 0.05;
        double max_task_step_m = 0.02;
        double max_task_step_rad = 0.1;
        double position_weight = 1.0;
        double orientation_weight = 0.1;
        bool nullspace_enabled = true;
        double nullspace_posture_gain = 0.05;
        int max_consecutive_failures = 20;
        bool failure_hold = true;
    } ik;

    struct TelemetryConfig {
        int stale_timeout_ms = 100;
        int anchor_reset_timeout_ms = 300;
    } tel;

    struct HumanModel {
        bool enabled = true;
        double shoulder_width_m = 0.38;
        double shoulder_down_offset_m = 0.18;
        double shoulder_back_offset_m = 0.05;
        double upper_arm_length_m = 0.30;
        double forearm_length_m = 0.27;
        double neutral_swivel_right_rad = 0.0;
        double neutral_swivel_left_rad = 0.0;
    } human;

    struct SwivelConfig {
        bool enabled = true;
        bool apply_to_ik = true;
        bool log_only = false;
        double neutral_gain = 0.2;
        double continuity_gain = 0.7;
        double wrist_orientation_gain = 0.1;
        double max_swivel_rate_rad_s = 1.0;
    } swivel;

    struct RateLimit {
        bool enabled = true;
        std::array<double, 8> max_step_rad_r = {0.035, 0.035, 0.035, 0.035, 0.035, 0.025, 0.035, 0.02};
        std::array<double, 8> max_step_rad_l = {0.035, 0.035, 0.035, 0.035, 0.035, 0.025, 0.035, 0.02};
    } rate_limit;

    struct DebugConfig {
        bool ik_debug = false;
        double ik_debug_rate_hz = 5.0;
        double ik_debug_burst_sec = 3.0;
    } debug;
};

VrTeleopConfig load_config(const std::string& path) {
    VrTeleopConfig cfg;
    try {
        YAML::Node node = YAML::LoadFile(path);
        if (node["vr_mapping"]) {
            auto m = node["vr_mapping"];
            if (m["position_scale_xyz"]) {
                for(int i=0; i<3; i++) cfg.mapping.position_scale_xyz[i] = m["position_scale_xyz"][i].as<double>();
            }
            if (m["max_delta_m"]) cfg.mapping.max_delta_m = m["max_delta_m"].as<double>();
            if (m["deadband_m"]) cfg.mapping.deadband_m = m["deadband_m"].as<double>();
            if (m["max_delta_per_cycle_m"]) cfg.mapping.max_delta_per_cycle_m = m["max_delta_per_cycle_m"].as<double>();
            auto load_matrix = [&](const std::string& key, Eigen::Matrix3d& mat) {
                if (m[key]) {
                    for(int i=0; i<3; i++) for(int j=0; j<3; j++) mat(i,j) = m[key][i][j].as<double>();
                }
            };
            load_matrix("right_translation_matrix", cfg.mapping.right_translation_matrix);
            load_matrix("left_translation_matrix", cfg.mapping.left_translation_matrix);
            load_matrix("right_axis_matrix", cfg.mapping.right_translation_matrix); // Alias
            load_matrix("left_axis_matrix", cfg.mapping.left_translation_matrix);   // Alias
            load_matrix("right_rotation_matrix", cfg.mapping.right_rotation_matrix);
            load_matrix("left_rotation_matrix", cfg.mapping.left_rotation_matrix);
            if (m["invert_rotation_delta"]) cfg.mapping.invert_rotation_delta = m["invert_rotation_delta"].as<bool>();
            if (m["rotation_compose_order"]) cfg.mapping.rotation_compose_order = m["rotation_compose_order"].as<std::string>();
        }
        if (node["ik"]) {
            auto i = node["ik"];
            if (i["mode"]) cfg.ik.mode = i["mode"].as<std::string>();
            if (i["max_iterations"]) cfg.ik.max_iterations = i["max_iterations"].as<int>();
            if (i["convergence_tol_m"]) cfg.ik.convergence_tol_m = i["convergence_tol_m"].as<double>();
            if (i["convergence_tol_pos_m"]) cfg.ik.convergence_tol_m = i["convergence_tol_pos_m"].as<double>();
            if (i["convergence_tol_rad"]) cfg.ik.convergence_tol_rad = i["convergence_tol_rad"].as<double>();
            if (i["convergence_tol_ori_rad"]) cfg.ik.convergence_tol_rad = i["convergence_tol_ori_rad"].as<double>();
            if (i["damping"]) cfg.ik.damping = i["damping"].as<double>();
            if (i["max_dq_norm"]) cfg.ik.max_dq_norm = i["max_dq_norm"].as<double>();
            if (i["max_task_step_m"]) cfg.ik.max_task_step_m = i["max_task_step_m"].as<double>();
            if (i["max_task_step_rad"]) cfg.ik.max_task_step_rad = i["max_task_step_rad"].as<double>();
            if (i["position_weight"]) cfg.ik.position_weight = i["position_weight"].as<double>();
            if (i["orientation_weight"]) cfg.ik.orientation_weight = i["orientation_weight"].as<double>();
            if (i["nullspace_enabled"]) cfg.ik.nullspace_enabled = i["nullspace_enabled"].as<bool>();
            if (i["nullspace_posture_gain"]) cfg.ik.nullspace_posture_gain = i["nullspace_posture_gain"].as<double>();
            if (i["max_consecutive_failures"]) cfg.ik.max_consecutive_failures = i["max_consecutive_failures"].as<int>();
            if (i["failure_hold"]) cfg.ik.failure_hold = i["failure_hold"].as<bool>();
        }
        if (node["telemetry"]) {
            auto t = node["telemetry"];
            if (t["stale_timeout_ms"]) cfg.tel.stale_timeout_ms = t["stale_timeout_ms"].as<int>();
            if (t["anchor_reset_timeout_ms"]) cfg.tel.anchor_reset_timeout_ms = t["anchor_reset_timeout_ms"].as<int>();
        }
        if (node["human_model"]) {
            auto h = node["human_model"];
            if (h["enabled"]) cfg.human.enabled = h["enabled"].as<bool>();
            if (h["shoulder_width_m"]) cfg.human.shoulder_width_m = h["shoulder_width_m"].as<double>();
            if (h["shoulder_down_offset_m"]) cfg.human.shoulder_down_offset_m = h["shoulder_down_offset_m"].as<double>();
            if (h["shoulder_back_offset_m"]) cfg.human.shoulder_back_offset_m = h["shoulder_back_offset_m"].as<double>();
            if (h["upper_arm_length_m"]) cfg.human.upper_arm_length_m = h["upper_arm_length_m"].as<double>();
            if (h["forearm_length_m"]) cfg.human.forearm_length_m = h["forearm_length_m"].as<double>();
            if (h["neutral_swivel_right_rad"]) cfg.human.neutral_swivel_right_rad = h["neutral_swivel_right_rad"].as<double>();
            if (h["neutral_swivel_left_rad"]) cfg.human.neutral_swivel_left_rad = h["neutral_swivel_left_rad"].as<double>();
        }
        if (node["swivel_prior"]) {
            auto s = node["swivel_prior"];
            if (s["enabled"]) cfg.swivel.enabled = s["enabled"].as<bool>();
            if (s["apply_to_ik"]) cfg.swivel.apply_to_ik = s["apply_to_ik"].as<bool>();
            if (s["log_only"]) cfg.swivel.log_only = s["log_only"].as<bool>();
            if (s["neutral_gain"]) cfg.swivel.neutral_gain = s["neutral_gain"].as<double>();
            if (s["continuity_gain"]) cfg.swivel.continuity_gain = s["continuity_gain"].as<double>();
            if (s["wrist_orientation_gain"]) cfg.swivel.wrist_orientation_gain = s["wrist_orientation_gain"].as<double>();
            if (s["max_swivel_rate_rad_s"]) cfg.swivel.max_swivel_rate_rad_s = s["max_swivel_rate_rad_s"].as<double>();
        }
        if (node["rate_limit"]) {
            auto r = node["rate_limit"];
            if (r["enabled"]) cfg.rate_limit.enabled = r["enabled"].as<bool>();
            if (r["max_step_rad"]) {
                auto ms = r["max_step_rad"];
                if (ms["right_arm"]) for (int k = 0; k < 8; ++k) cfg.rate_limit.max_step_rad_r[k] = ms["right_arm"][k].as<double>();
                if (ms["left_arm"]) for (int k = 0; k < 8; ++k) cfg.rate_limit.max_step_rad_l[k] = ms["left_arm"][k].as<double>();
            }
        }
        if (node["debug"]) {
            auto d = node["debug"];
            if (d["ik_debug"]) cfg.debug.ik_debug = d["ik_debug"].as<bool>();
            if (d["ik_debug_rate_hz"]) cfg.debug.ik_debug_rate_hz = d["ik_debug_rate_hz"].as<double>();
            if (d["ik_debug_burst_sec"]) cfg.debug.ik_debug_burst_sec = d["ik_debug_burst_sec"].as<double>();
        }
    } catch (const std::exception& e) {
        LOG_WARN("Failed to load config " << path << ": " << e.what() << ". Using defaults.");
    }
    return cfg;
}

struct ArmAnchor {
    bool active = false;
    std::array<double, 3> robot_ee_pos_anchor = {0, 0, 0};
    std::array<double, 4> robot_ee_quat_anchor = {0, 0, 0, 1};
    std::array<double, 7> robot_q_anchor = {0,0,0,0,0,0,0};
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::string follower_ip = "127.0.0.1";
    uint16_t right_port = 50000;
    uint16_t left_port = 50001;
    uint16_t vr_port = 54002;
    uint16_t telemetry_port = 51000;
    uint16_t viz_port = 54001;
    double rate_hz = 100.0; // IK loop rate
    std::string right_urdf = "urdf/openarm_right.urdf";
    std::string left_urdf = "urdf/openarm_left.urdf";
    std::string right_ee = "openarm_right_hand";
    std::string left_ee = "openarm_left_hand";

    std::string config_path = "config/vr_teleop_config.yaml";
    bool ik_debug = false;
    bool dry_run = false;
    bool mapping_test = false;
    double max_test_sec = 0.0;
    std::string log_csv_path;
    std::vector<std::string> set_overrides;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--follower-ip" && i + 1 < argc) follower_ip = argv[++i];
        else if (arg == "--vr-port" && i + 1 < argc) vr_port = std::stoi(argv[++i]);
        else if (arg == "--telemetry-port" && i + 1 < argc) telemetry_port = std::stoi(argv[++i]);
        else if (arg == "--rate-hz" && i + 1 < argc) rate_hz = std::stod(argv[++i]);
        else if (arg == "--right-urdf" && i + 1 < argc) right_urdf = argv[++i];
        else if (arg == "--left-urdf" && i + 1 < argc) left_urdf = argv[++i];
        else if (arg == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (arg == "--ik-debug") ik_debug = true;
        else if (arg == "--ik-debug-rate-hz" && i + 1 < argc) set_overrides.push_back(std::string("debug.ik_debug_rate_hz=") + argv[++i]);
        else if (arg == "--ik-debug-burst-sec" && i + 1 < argc) set_overrides.push_back(std::string("debug.ik_debug_burst_sec=") + argv[++i]);
        else if (arg == "--log-csv" && i + 1 < argc) log_csv_path = argv[++i];
        else if (arg == "--max-test-sec" && i + 1 < argc) max_test_sec = std::stod(argv[++i]);
        else if (arg == "--dry-run") dry_run = true;
        else if (arg == "--vr-mapping-test") mapping_test = true;
        else if (arg == "--pose-ik") { /* handled after load_config */ }
        else if (arg == "--position-ik") { /* handled after load_config */ }
        else if (arg == "--set" && i + 1 < argc) set_overrides.push_back(argv[++i]);
    }

    VrTeleopConfig cfg = load_config(config_path);
    if (ik_debug) cfg.debug.ik_debug = true; // Override by CLI
    if (!log_csv_path.empty()) cfg.debug.ik_debug = true;

    auto parse_bool = [](const std::string& value) {
        return value == "1" || value == "true" || value == "TRUE" || value == "yes" || value == "on";
    };
    auto apply_set = [&](const std::string& item) {
        auto eq = item.find('=');
        if (eq == std::string::npos) {
            LOG_WARN("Ignoring malformed --set override: " << item);
            return;
        }
        std::string key = item.substr(0, eq);
        std::string value = item.substr(eq + 1);
        if (key == "ik.orientation_weight") cfg.ik.orientation_weight = std::stod(value);
        else if (key == "ik.position_weight") cfg.ik.position_weight = std::stod(value);
        else if (key == "ik.damping") cfg.ik.damping = std::stod(value);
        else if (key == "ik.max_dq_norm") cfg.ik.max_dq_norm = std::stod(value);
        else if (key == "ik.mode") cfg.ik.mode = value;
        else if (key == "human_model.enabled") cfg.human.enabled = parse_bool(value);
        else if (key == "swivel_prior.enabled") cfg.swivel.enabled = parse_bool(value);
        else if (key == "swivel_prior.apply_to_ik") cfg.swivel.apply_to_ik = parse_bool(value);
        else if (key == "swivel_prior.log_only") cfg.swivel.log_only = parse_bool(value);
        else if (key == "vr_mapping.rotation_compose_order") cfg.mapping.rotation_compose_order = value;
        else if (key == "vr_mapping.invert_rotation_delta") cfg.mapping.invert_rotation_delta = parse_bool(value);
        else if (key == "debug.ik_debug_rate_hz") cfg.debug.ik_debug_rate_hz = std::stod(value);
        else if (key == "debug.ik_debug_burst_sec") cfg.debug.ik_debug_burst_sec = std::stod(value);
        else LOG_WARN("Unknown --set override key: " << key);
    };
    for (const auto& item : set_overrides) apply_set(item);
    
    // Command line overrides for IK mode
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--pose-ik") cfg.ik.mode = "pose";
        else if (arg == "--position-ik") cfg.ik.mode = "position";
    }

    LOG_INFO("Starting VR Bimanual Leader");
    LOG_INFO("Follower IP: " << follower_ip << " (Ports: " << right_port << ", " << left_port << ")");
    LOG_INFO("VR Port: " << vr_port << " | Telemetry Port: " << telemetry_port);
    LOG_INFO("OpenXR basis assumption: +X=right, +Y=up, -Z=forward. Robot base assumption: +X=forward, +Y=left, +Z=up.");
    log_mapping_basis("RIGHT translation", cfg.mapping.right_translation_matrix);
    log_mapping_basis("LEFT translation", cfg.mapping.left_translation_matrix);
    log_mapping_basis("RIGHT rotation", cfg.mapping.right_rotation_matrix);
    log_mapping_basis("LEFT rotation", cfg.mapping.left_rotation_matrix);
    LOG_INFO("Rotation delta inversion: " << (cfg.mapping.invert_rotation_delta ? "ON" : "OFF"));

    std::ofstream csv_log;
    if (!log_csv_path.empty()) {
        csv_log.open(log_csv_path);
        if (!csv_log) {
            LOG_ERROR("Failed to open IK CSV log: " << log_csv_path);
            return 1;
        }
        csv_log << "timestamp_sec,side,packet_version,grip,trigger,telemetry_age_ms,"
                << "delta_pos_openxr_x,delta_pos_openxr_y,delta_pos_openxr_z,"
                << "delta_pos_robot_x,delta_pos_robot_y,delta_pos_robot_z,"
                << "delta_rot_openxr_axis_x,delta_rot_openxr_axis_y,delta_rot_openxr_axis_z,delta_rot_openxr_angle,"
                << "delta_rot_robot_axis_x,delta_rot_robot_axis_y,delta_rot_robot_axis_z,delta_rot_robot_angle,"
                << "target_pos_x,target_pos_y,target_pos_z,current_pos_x,current_pos_y,current_pos_z,"
                << "target_quat_x,target_quat_y,target_quat_z,target_quat_w,current_quat_x,current_quat_y,current_quat_z,current_quat_w,"
                << "position_error_norm,orientation_error_norm,"
                << "q_feedback_1,q_feedback_2,q_feedback_3,q_feedback_4,q_feedback_5,q_feedback_6,q_feedback_7,q_feedback_gripper,"
                << "q_ik_1,q_ik_2,q_ik_3,q_ik_4,q_ik_5,q_ik_6,q_ik_7,q_ik_gripper,"
                << "q_cmd_1,q_cmd_2,q_cmd_3,q_cmd_4,q_cmd_5,q_cmd_6,q_cmd_7,q_cmd_gripper,"
                << "ik_success,ik_iterations,dq_norm,rate_limited,joint_limit_clamped,nan_rejected,telemetry_stale,"
                << "human_model_enabled,swivel_prior_enabled,swivel_right,swivel_left,swivel_applied,"
                << "orientation_weight,position_weight,damping,rotation_compose_order\n";
    }

    // IK Solvers
    std::vector<std::string> joint_names_r = {
        "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3", 
        "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7"
    };
    std::vector<std::string> joint_names_l = {
        "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3", 
        "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6", "openarm_left_joint7"
    };
    
    OpenArmPinocchioIK ik_r(right_urdf, right_ee, joint_names_r);
    OpenArmPinocchioIK ik_l(left_urdf, left_ee, joint_names_l);

    // UDP
    net::UdpSender sender_r(follower_ip, right_port);
    net::UdpSender sender_l(follower_ip, left_port);
    net::UdpSender viz_sender("127.0.0.1", viz_port);

    // VR Data
    struct VrState {
        uint32_t seq = 0;
        bool enabled = false;
        bool estop = false;
        bool recenter = false;
        struct Arm {
            Eigen::Vector3d delta_pos_openxr = Eigen::Vector3d::Zero();
            Eigen::Quaterniond delta_rot_openxr = Eigen::Quaterniond::Identity();
            Eigen::Vector3d delta_pos_robot = Eigen::Vector3d::Zero();
            Eigen::Quaterniond delta_rot_robot = Eigen::Quaterniond::Identity();
            float grip = 0;
            float trigger = 0;
        } left, right;
        Eigen::Vector3d hmd_pos = Eigen::Vector3d::Zero();
        Eigen::Quaterniond hmd_quat = Eigen::Quaterniond::Identity();
        int version = 0;
    };
    VrState current_vr_state;
    std::mutex vr_mutex;
    bool vr_connected = false;
    std::chrono::steady_clock::time_point last_vr_time;

    net::UdpReceiver vr_receiver("0.0.0.0", vr_port);
    vr_receiver.start_raw([&](const void* data, size_t size) {
        if (size < 8) return;
        uint32_t magic = *static_cast<const uint32_t*>(data);
        
        std::lock_guard<std::mutex> lock(vr_mutex);
        if (magic == net::VrRelativeTeleopPacketV2::MAGIC && size == sizeof(net::VrRelativeTeleopPacketV2)) {
            const net::VrRelativeTeleopPacketV2* pkt = static_cast<const net::VrRelativeTeleopPacketV2*>(data);
            current_vr_state.version = 2;
            current_vr_state.seq = pkt->seq;
            current_vr_state.enabled = (pkt->left_grip > 0.1 || pkt->right_grip > 0.1);
            current_vr_state.estop = false; // V2 can add estop field if needed
            current_vr_state.recenter = false; // V2 uses grip anchor logic in sender
            
            auto fill_arm_v2 = [&](const float* p, const float* r, float g, float t, VrState::Arm& arm, const Eigen::Matrix3d& trans_mat, const Eigen::Matrix3d& rot_mat) {
                arm.delta_pos_openxr = Eigen::Vector3d(p[0], p[1], p[2]);
                arm.delta_rot_openxr = Eigen::Quaterniond(r[3], r[0], r[1], r[2]);
                
                // MAPPING (Phase 3)
                Eigen::Vector3d scale(cfg.mapping.position_scale_xyz[0], cfg.mapping.position_scale_xyz[1], cfg.mapping.position_scale_xyz[2]);
                arm.delta_pos_robot = trans_mat * (scale.array() * arm.delta_pos_openxr.array()).matrix();
                arm.delta_rot_robot = Eigen::Quaterniond(rot_mat * arm.delta_rot_openxr.toRotationMatrix() * rot_mat.transpose());
                if (cfg.mapping.invert_rotation_delta) {
                    arm.delta_rot_robot = arm.delta_rot_robot.conjugate();
                }
                
                arm.grip = g;
                arm.trigger = t;
            };
            fill_arm_v2(pkt->left_delta_pos_openxr, pkt->left_delta_rot_openxr_xyzw, pkt->left_grip, pkt->left_trigger, current_vr_state.left, cfg.mapping.left_translation_matrix, cfg.mapping.left_rotation_matrix);
            fill_arm_v2(pkt->right_delta_pos_openxr, pkt->right_delta_rot_openxr_xyzw, pkt->right_grip, pkt->right_trigger, current_vr_state.right, cfg.mapping.right_translation_matrix, cfg.mapping.right_rotation_matrix);
            
            current_vr_state.hmd_pos = Eigen::Vector3d(pkt->hmd_pos_openxr[0], pkt->hmd_pos_openxr[1], pkt->hmd_pos_openxr[2]);
            current_vr_state.hmd_quat = Eigen::Quaterniond(pkt->hmd_quat_openxr_xyzw[3], pkt->hmd_quat_openxr_xyzw[0], pkt->hmd_quat_openxr_xyzw[1], pkt->hmd_quat_openxr_xyzw[2]);
            
            vr_connected = true;
            last_vr_time = std::chrono::steady_clock::now();
        }
        
        // Forward RAW VR packet to visualization bridge.
        viz_sender.send_raw(data, size);
    });

    // Telemetry Data (Robot Feedback for Sync)
    std::mutex tel_mutex;
    std::array<double, 7> robot_q_r_fb = {0,0,0,0,0,0,0};
    std::array<double, 7> robot_q_l_fb = {0,0,0,0,0,0,0};
    std::chrono::steady_clock::time_point last_tel_time;
    bool tel_connected = false;

    net::UdpReceiver tel_receiver("0.0.0.0", telemetry_port);
    tel_receiver.start_raw([&](const void* data, size_t size) {
        if (size == sizeof(telemetry::OpenArmTelemetryPacketV1)) {
            const auto* pkt = static_cast<const telemetry::OpenArmTelemetryPacketV1*>(data);
            if (pkt->magic == telemetry::TELEMETRY_MAGIC) {
                std::lock_guard<std::mutex> lock(tel_mutex);
                for (int i = 0; i < 7; ++i) robot_q_r_fb[i] = pkt->observation_state[i];
                for (int i = 0; i < 7; ++i) robot_q_l_fb[i] = pkt->observation_state[i + 8];
                last_tel_time = std::chrono::steady_clock::now();
                tel_connected = true;
            }
        }
    });

    // Robot State (Target joints)
    std::array<double, 7> target_q_r = {0,0,0,0,0,0,0}; 
    std::array<double, 7> target_q_l = {0,0,0,0,0,0,0};
    double target_grip_r = 0, target_grip_l = 0;

    ArmAnchor anchor_r, anchor_l;

    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    auto next_time = std::chrono::steady_clock::now();
    auto last_log_time = std::chrono::steady_clock::now();
    auto start_time = std::chrono::steady_clock::now();
    uint32_t seq = 0;

    // Sync parameters to IK objects
    OpenArmPinocchioIK::IKParams ik_p;
    ik_p.max_iterations = cfg.ik.max_iterations;
    ik_p.convergence_tol_m = cfg.ik.convergence_tol_m;
    ik_p.convergence_tol_rad = cfg.ik.convergence_tol_rad;
    ik_p.damping = cfg.ik.damping;
    ik_p.max_dq_norm = cfg.ik.max_dq_norm;
    ik_p.max_task_step_m = cfg.ik.max_task_step_m;
    ik_p.max_task_step_rad = cfg.ik.max_task_step_rad;
    ik_p.position_weight = cfg.ik.position_weight;
    ik_p.orientation_weight = cfg.ik.orientation_weight;
    ik_p.nullspace_enabled = cfg.ik.nullspace_enabled;
    ik_p.nullspace_posture_gain = cfg.ik.nullspace_posture_gain;
    ik_r.set_params(ik_p);
    ik_l.set_params(ik_p);

    // Swivel Estimators
    control::TemporalSwivelEstimator::Config swivel_cfg;
    swivel_cfg.enabled = cfg.swivel.enabled;
    swivel_cfg.neutral_gain = cfg.swivel.neutral_gain;
    swivel_cfg.continuity_gain = cfg.swivel.continuity_gain;
    swivel_cfg.wrist_orientation_gain = cfg.swivel.wrist_orientation_gain;
    swivel_cfg.max_swivel_rate_rad_s = cfg.swivel.max_swivel_rate_rad_s;

    swivel_cfg.neutral_swivel_rad = cfg.human.neutral_swivel_right_rad;
    control::TemporalSwivelEstimator swivel_r(swivel_cfg);
    
    swivel_cfg.neutral_swivel_rad = cfg.human.neutral_swivel_left_rad;
    control::TemporalSwivelEstimator swivel_l(swivel_cfg);

    LOG_INFO("Entering control loop at " << rate_hz << " Hz");

    while (keep_running) {
        VrState vr_pkt;
        {
            std::lock_guard<std::mutex> lock(vr_mutex);
            vr_pkt = current_vr_state;
        }

        std::array<double, 7> fb_r, fb_l;
        bool telemetry_stale = false;
        double telemetry_age_ms = -1.0;
        {
            std::lock_guard<std::mutex> lock(tel_mutex);
            fb_r = robot_q_r_fb;
            fb_l = robot_q_l_fb;
            
            auto now = std::chrono::steady_clock::now();
            auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tel_time).count();
            telemetry_age_ms = tel_connected ? static_cast<double>(age_ms) : -1.0;
            if (tel_connected && age_ms > cfg.tel.stale_timeout_ms) {
                telemetry_stale = true;
            }
            if (tel_connected && age_ms > cfg.tel.anchor_reset_timeout_ms) {
                anchor_r.active = false;
                anchor_l.active = false;
            }
        }

        bool right_ik_success = false;
        bool left_ik_success = false;

        auto process_arm_logic = [&](net::ArmSide side, const VrState& vr_pkt,
                                    OpenArmPinocchioIK& ik, ArmAnchor& anchor, control::TemporalSwivelEstimator& swivel_est,
                                    std::array<double, 7>& q_target, 
                                     double& grip_target, const std::array<double, 7>& q_feedback, bool& ik_success, 
                                     double& burst_timer) {
            
            if (telemetry_stale) {
                ik_success = false;
                return; 
            }

            const VrState::Arm& arm_state = (side == net::ArmSide::RIGHT) ? vr_pkt.right : vr_pkt.left;
            grip_target = arm_state.trigger;

            bool feedback_available = tel_connected || dry_run;
            if (arm_state.grip > 0.5 && feedback_available) {
                if (!anchor.active || vr_pkt.recenter) {
                    ik.compute_fk_pose(q_feedback, anchor.robot_ee_pos_anchor, anchor.robot_ee_quat_anchor); 
                    anchor.robot_q_anchor = q_feedback;
                    anchor.active = true;
                    LOG_INFO((side==net::ArmSide::RIGHT?"RIGHT":"LEFT") << " Anchor updated. EE Pos: " << std::fixed << std::setprecision(3) 
                             << anchor.robot_ee_pos_anchor[0] << "," << anchor.robot_ee_pos_anchor[1] << "," << anchor.robot_ee_pos_anchor[2]);
                    burst_timer = cfg.debug.ik_debug_burst_sec;
                }

                // 1. Get Delta from VrState (already robot-space)
                Eigen::Vector3d delta_pos_robot = arm_state.delta_pos_robot;

                // Clamp delta pos to avoid runaway
                double d_pos_norm = delta_pos_robot.norm();
                if (d_pos_norm > cfg.mapping.max_delta_m) {
                    delta_pos_robot *= (cfg.mapping.max_delta_m / d_pos_norm);
                }
                
                Eigen::Quaterniond delta_rot_robot = arm_state.delta_rot_robot;

                std::array<double, 3> x_des = {
                    anchor.robot_ee_pos_anchor[0] + delta_pos_robot.x(),
                    anchor.robot_ee_pos_anchor[1] + delta_pos_robot.y(),
                    anchor.robot_ee_pos_anchor[2] + delta_pos_robot.z()
                };

                Eigen::Quaterniond q_anchor(anchor.robot_ee_quat_anchor[3], anchor.robot_ee_quat_anchor[0], anchor.robot_ee_quat_anchor[1], anchor.robot_ee_quat_anchor[2]);
                
                Eigen::Quaterniond quat_des;
                if (cfg.mapping.rotation_compose_order == "delta_then_anchor") {
                    quat_des = delta_rot_robot * q_anchor;
                } else {
                    quat_des = q_anchor * delta_rot_robot;
                }

                // 3. Swivel Estimation (Phase 7)
                double swivel_target = 0.0;
                if (cfg.human.enabled && cfg.swivel.enabled) {
                    Eigen::Vector3d hmd_p = vr_pkt.hmd_pos;
                    Eigen::Quaterniond hmd_q = vr_pkt.hmd_quat;
                    
                    double w = cfg.human.shoulder_width_m * (side == net::ArmSide::RIGHT ? 0.5 : -0.5);
                    Eigen::Vector3d shoulder_local(w, -cfg.human.shoulder_down_offset_m, -cfg.human.shoulder_back_offset_m);
                    Eigen::Vector3d shoulder_pos_vr = hmd_p + hmd_q * shoulder_local;
                    
                    Eigen::Vector3d wrist_pos_robot(x_des[0], x_des[1], x_des[2]);
                    
                    // Temporary: use a fixed shoulder position relative to robot base for now
                    Eigen::Vector3d shoulder_pos_robot(0.0, (side == net::ArmSide::RIGHT ? -0.2 : 0.2), 0.5); 
                    
                    control::SwivelInput s_in;
                    s_in.shoulder_pos = shoulder_pos_robot;
                    s_in.wrist_pos = wrist_pos_robot;
                    s_in.wrist_orientation = quat_des; // Use composed orientation
                    s_in.dt = 1.0 / rate_hz;
                    s_in.right_arm = (side == net::ArmSide::RIGHT);
                    
                    swivel_target = swivel_est.update(s_in);
                }

                if (mapping_test) {
                    static int mt_cnt = 0;
                    if (mt_cnt++ % 50 == 0) {
                        Eigen::AngleAxisd aa(delta_rot_robot);
                        std::cout << "[MAP TEST] " << (side==net::ArmSide::RIGHT?"R":"L") 
                                  << " Delta_pos: " << delta_pos_robot.transpose()
                                  << " Delta_rot (AA): " << aa.axis().transpose() << " | " << aa.angle()
                                  << " Swivel: " << swivel_target << std::endl;
                    }
                }

                std::array<double, 4> quat_des_arr = { quat_des.x(), quat_des.y(), quat_des.z(), quat_des.w() };

                std::array<double, 7> q_anchor_with_swivel = anchor.robot_q_anchor;
                // Heuristic: Joint 3 (index 2) typically controls the swivel rotation for this 7-DOF kinematic structure.
                // We offset it by the estimated swivel angle to bias the IK towards a natural posture.
                bool swivel_applied = cfg.human.enabled && cfg.swivel.enabled && cfg.swivel.apply_to_ik && !cfg.swivel.log_only;
                if (swivel_applied) {
                    q_anchor_with_swivel[2] += swivel_target; 
                }

                std::array<double, 7> q_ik_out = q_feedback;
                bool solved = false;
                double residual_pos = 0, residual_ori = 0;
                if (cfg.ik.mode == "pose") {
                    solved = ik.solve_pose(x_des, quat_des_arr, q_feedback, q_ik_out, &q_anchor_with_swivel);
                } else {
                    solved = ik.solve(x_des, q_feedback, q_ik_out, &q_anchor_with_swivel);
                }

                std::array<double, 3> actual_pos;
                std::array<double, 4> actual_quat;
                ik.compute_fk_pose(q_ik_out, actual_pos, actual_quat);
                residual_pos = std::sqrt(std::pow(actual_pos[0]-x_des[0],2) + std::pow(actual_pos[1]-x_des[1],2) + std::pow(actual_pos[2]-x_des[2],2));
                Eigen::Quaterniond q_actual(actual_quat[3], actual_quat[0], actual_quat[1], actual_quat[2]);
                residual_ori = Eigen::AngleAxisd(q_actual.conjugate() * quat_des).angle();

                bool rate_limited = false;
                if (solved) {
                    
                    const auto& max_steps = (side == net::ArmSide::RIGHT) ? cfg.rate_limit.max_step_rad_r : cfg.rate_limit.max_step_rad_l;
                    
                    std::array<double, 7> q_limited;
                    for (int i = 0; i < 7; ++i) {
                        double diff = q_ik_out[i] - q_target[i];
                        if (cfg.rate_limit.enabled && std::abs(diff) > max_steps[i]) {
                            diff = (diff > 0 ? 1.0 : -1.0) * max_steps[i];
                            rate_limited = true;
                        }
                        q_limited[i] = q_target[i] + diff;
                    }

                    q_target = q_limited;
                    ik_success = true;
                }
                // Phase 4 Logging
                if (cfg.debug.ik_debug) {
                    bool should_log = false;
                    static std::chrono::steady_clock::time_point last_t_r, last_t_l;
                    auto& last_t = (side == net::ArmSide::RIGHT) ? last_t_r : last_t_l;
                    auto now = std::chrono::steady_clock::now();
                    double log_interval = (burst_timer > 0) ? 0.05 : (1.0 / cfg.debug.ik_debug_rate_hz); // 20Hz in burst, otherwise cfg rate

                    if (std::chrono::duration<double>(now - last_t).count() > log_interval) {
                        should_log = true;
                        last_t = now;
                    }

                    if (should_log) {
                        std::stringstream log_ss;
                        log_ss << "--- IK DEBUG (" << (side == net::ArmSide::RIGHT ? "RIGHT" : "LEFT") << ") ---";
                        if (vr_pkt.version == 2) {
                            Eigen::AngleAxisd raw_aa(arm_state.delta_rot_openxr);
                            log_ss << "\n  [Packet V2 Raw] Pos: " << arm_state.delta_pos_openxr.transpose() 
                                   << " | Rot (AA): " << raw_aa.axis().transpose() << " | " << raw_aa.angle() << " rad";
                        }
                        Eigen::AngleAxisd mapped_aa(arm_state.delta_rot_robot);
                        log_ss << "\n  [Mapped Robot]  Pos: " << arm_state.delta_pos_robot.transpose() 
                               << " | Rot (AA): " << mapped_aa.axis().transpose() << " | " << mapped_aa.angle() << " rad";
                        
                        log_ss << "\n  [Target Pose]   Pos: " << x_des[0] << "," << x_des[1] << "," << x_des[2]
                               << "\n  [IK Status]     Success: " << (solved ? "YES" : "NO") << " | Residual Pos Err: " << residual_pos << "m";
                        
                        LOG_INFO(log_ss.str());
                        if (csv_log) {
                            Eigen::AngleAxisd raw_aa(arm_state.delta_rot_openxr);
                            Eigen::AngleAxisd mapped_aa(arm_state.delta_rot_robot);
                            double dq_norm = 0.0;
                            bool nan_rejected = false;
                            for (int i = 0; i < 7; ++i) {
                                double d = q_ik_out[i] - q_feedback[i];
                                dq_norm += d * d;
                                if (!std::isfinite(q_target[i]) || !std::isfinite(q_ik_out[i])) nan_rejected = true;
                            }
                            dq_norm = std::sqrt(dq_norm);
                            csv_log << std::fixed << std::setprecision(9)
                                    << openarm_wifi_teleop::utils::now_ns() * 1e-9 << ','
                                    << (side == net::ArmSide::RIGHT ? "right" : "left") << ','
                                    << vr_pkt.version << ','
                                    << arm_state.grip << ',' << arm_state.trigger << ',' << telemetry_age_ms << ','
                                    << arm_state.delta_pos_openxr.x() << ',' << arm_state.delta_pos_openxr.y() << ',' << arm_state.delta_pos_openxr.z() << ','
                                    << delta_pos_robot.x() << ',' << delta_pos_robot.y() << ',' << delta_pos_robot.z() << ','
                                    << raw_aa.axis().x() << ',' << raw_aa.axis().y() << ',' << raw_aa.axis().z() << ',' << raw_aa.angle() << ','
                                    << mapped_aa.axis().x() << ',' << mapped_aa.axis().y() << ',' << mapped_aa.axis().z() << ',' << mapped_aa.angle() << ','
                                    << x_des[0] << ',' << x_des[1] << ',' << x_des[2] << ','
                                    << actual_pos[0] << ',' << actual_pos[1] << ',' << actual_pos[2] << ','
                                    << quat_des.x() << ',' << quat_des.y() << ',' << quat_des.z() << ',' << quat_des.w() << ','
                                    << actual_quat[0] << ',' << actual_quat[1] << ',' << actual_quat[2] << ',' << actual_quat[3] << ','
                                    << residual_pos << ',' << residual_ori << ',';
                            for (int i = 0; i < 7; ++i) csv_log << q_feedback[i] << ',';
                            csv_log << grip_target << ',';
                            for (int i = 0; i < 7; ++i) csv_log << q_ik_out[i] << ',';
                            csv_log << grip_target << ',';
                            for (int i = 0; i < 7; ++i) csv_log << q_target[i] << ',';
                            csv_log << grip_target << ','
                                    << (solved ? 1 : 0) << ',' << cfg.ik.max_iterations << ',' << dq_norm << ','
                                    << (rate_limited ? 1 : 0) << ",0," << (nan_rejected ? 1 : 0) << ',' << (telemetry_stale ? 1 : 0) << ','
                                    << (cfg.human.enabled ? 1 : 0) << ',' << (cfg.swivel.enabled ? 1 : 0) << ','
                                    << (side == net::ArmSide::RIGHT ? swivel_target : 0.0) << ','
                                    << (side == net::ArmSide::LEFT ? swivel_target : 0.0) << ','
                                    << (swivel_applied ? 1 : 0) << ','
                                    << cfg.ik.orientation_weight << ',' << cfg.ik.position_weight << ',' << cfg.ik.damping << ','
                                    << cfg.mapping.rotation_compose_order << '\n';
                        }
                    }
                    if (burst_timer > 0) burst_timer -= (1.0 / rate_hz);
                }
            } else {
                anchor.active = false;
                if (feedback_available) q_target = q_feedback;
            }
        };


        static double burst_timer_r = 0, burst_timer_l = 0;
        process_arm_logic(net::ArmSide::RIGHT, vr_pkt, ik_r, anchor_r, swivel_r, target_q_r, target_grip_r, fb_r, right_ik_success, burst_timer_r);
        process_arm_logic(net::ArmSide::LEFT, vr_pkt, ik_l, anchor_l, swivel_l, target_q_l, target_grip_l, fb_l, left_ik_success, burst_timer_l);

        // Send to Follower
        auto send_pkt = [&](net::UdpSender& sender, net::ArmSide side, const std::array<double, 7>& q, double grip, bool arm_enabled) {
            net::TeleopPacket pkt;
            std::memset(&pkt, 0, sizeof(pkt));
            pkt.magic = net::TELEOP_MAGIC;
            pkt.version = net::TELEOP_VERSION;
            pkt.packet_size = sizeof(net::TeleopPacket);
            pkt.seq = seq;
            pkt.arm_side = static_cast<uint8_t>(side);
            pkt.mode = static_cast<uint8_t>(net::ControlMode::UNILATERAL);
            bool enabled = (arm_enabled && tel_connected && !vr_pkt.estop && !telemetry_stale);
            if (dry_run) enabled = false;
            pkt.enable = enabled ? 1 : 0;
            pkt.arm_dof = 7;
            pkt.hand_dof = 1;
            
            // FINAL SAFETY: Do not send NaN to robot
            bool has_nan = false;
            for (int i = 0; i < 7; ++i) {
                if (std::isnan(q[i])) has_nan = true;
                pkt.arm_pos[i] = q[i];
            }
            pkt.hand_pos[0] = grip;

            if (has_nan) {
                pkt.enable = 0; // Disable if target is invalid
                static auto last_nan_log = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (now - last_nan_log > std::chrono::seconds(1)) {
                    LOG_ERROR("Refusing to send NaN target to " << (side == net::ArmSide::RIGHT ? "RIGHT" : "LEFT"));
                    last_nan_log = now;
                }
            }
            
            net::PacketCodec::encode(pkt);
            sender.send(pkt);
        };

        send_pkt(sender_r, net::ArmSide::RIGHT, target_q_r, target_grip_r, vr_pkt.right.grip > 0.5);
        send_pkt(sender_l, net::ArmSide::LEFT, target_q_l, target_grip_l, vr_pkt.left.grip > 0.5);

        // Logging (1Hz)
        auto now = std::chrono::steady_clock::now();
        if (now - last_log_time > std::chrono::seconds(1)) {
            std::cout << "\n--- VR Teleop Status ---" << std::endl;
            std::cout << "VR:  " << (vr_connected ? "OK" : "NO (Waiting for Quest2...)") << std::endl;
            std::cout << "TEL: " << (tel_connected ? (telemetry_stale ? "STALE (Timeout!)" : "OK") : "NO (Waiting for Follower --publish-telemetry ...)") << std::endl;
            
            auto log_arm = [&](const char* label, bool grip, const Eigen::Vector3d& delta, const std::array<double, 7>& q_fb, const std::array<double, 7>& q_target, bool ik_ok) {
                std::cout << label << ": " << (grip ? "[GRIP] " : "[IDLE] ");
                std::cout << "Delta: (" << std::fixed << std::setprecision(3) << delta.transpose() << ") ";
                if (grip) {
                    std::cout << "IK: " << (ik_ok ? "OK" : "FAIL") << std::endl;
                } else {
                    std::cout << "IK: ---" << std::endl;
                }
                std::cout << "  FB (j1-3): " << q_fb[0] << ", " << q_fb[1] << ", " << q_fb[2] << std::endl;
                std::cout << "  TG (j1-3): " << q_target[0] << ", " << q_target[1] << ", " << q_target[2] << std::endl;
            };

            log_arm("RIGHT", vr_pkt.right.grip > 0.5, vr_pkt.right.delta_pos_robot, fb_r, target_q_r, right_ik_success);
            log_arm("LEFT ", vr_pkt.left.grip > 0.5, vr_pkt.left.delta_pos_robot, fb_l, target_q_l, left_ik_success);
            
            if (vr_pkt.version == 2) {
                std::cout << "V2 RAW (R): (" << vr_pkt.right.delta_pos_openxr.transpose() << ")" << std::endl;
                std::cout << "V2 RAW (L): (" << vr_pkt.left.delta_pos_openxr.transpose() << ")" << std::endl;
            }
            
            last_log_time = now;
        }

        seq++;
        if (max_test_sec > 0.0 && std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() >= max_test_sec) {
            keep_running = false;
        }
        next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(period);
        std::this_thread::sleep_until(next_time);
    }

    vr_receiver.stop();
    tel_receiver.stop();
    return 0;
}
