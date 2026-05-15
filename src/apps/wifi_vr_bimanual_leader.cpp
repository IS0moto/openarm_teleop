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

struct VrTeleopConfig {
    struct VrMapping {
        std::array<double, 3> position_scale_xyz = {1,1,1};
        double max_delta_m = 0.30;
        double deadband_m = 0.002;
        double max_delta_per_cycle_m = 0.01;
        Eigen::Matrix3d right_axis_matrix = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d left_axis_matrix = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d right_rotation_matrix = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d left_rotation_matrix = Eigen::Matrix3d::Identity();
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
            cfg.mapping.max_delta_m = m["max_delta_m"].as<double>();
            cfg.mapping.deadband_m = m["deadband_m"].as<double>();
            cfg.mapping.max_delta_per_cycle_m = m["max_delta_per_cycle_m"].as<double>();
            auto load_matrix = [&](const std::string& key, Eigen::Matrix3d& mat) {
                if (m[key]) {
                    for(int i=0; i<3; i++) for(int j=0; j<3; j++) mat(i,j) = m[key][i][j].as<double>();
                }
            };
            load_matrix("right_axis_matrix", cfg.mapping.right_axis_matrix);
            load_matrix("left_axis_matrix", cfg.mapping.left_axis_matrix);
            load_matrix("right_rotation_matrix", cfg.mapping.right_rotation_matrix);
            load_matrix("left_rotation_matrix", cfg.mapping.left_rotation_matrix);
        }
        if (node["ik"]) {
            auto i = node["ik"];
            if (i["mode"]) cfg.ik.mode = i["mode"].as<std::string>();
            cfg.ik.max_iterations = i["max_iterations"].as<int>();
            cfg.ik.convergence_tol_m = i["convergence_tol_m"].as<double>();
            cfg.ik.convergence_tol_rad = i["convergence_tol_rad"].as<double>();
            cfg.ik.damping = i["damping"].as<double>();
            cfg.ik.max_dq_norm = i["max_dq_norm"].as<double>();
            cfg.ik.max_task_step_m = i["max_task_step_m"].as<double>();
            cfg.ik.max_task_step_rad = i["max_task_step_rad"].as<double>();
            cfg.ik.position_weight = i["position_weight"].as<double>();
            cfg.ik.orientation_weight = i["orientation_weight"].as<double>();
            cfg.ik.nullspace_enabled = i["nullspace_enabled"].as<bool>();
            cfg.ik.nullspace_posture_gain = i["nullspace_posture_gain"].as<double>();
            cfg.ik.max_consecutive_failures = i["max_consecutive_failures"].as<int>();
            cfg.ik.failure_hold = i["failure_hold"].as<bool>();
        }
        if (node["human_model"]) {
            auto h = node["human_model"];
            cfg.human.enabled = h["enabled"].as<bool>();
            cfg.human.shoulder_width_m = h["shoulder_width_m"].as<double>();
            cfg.human.shoulder_down_offset_m = h["shoulder_down_offset_m"].as<double>();
            cfg.human.shoulder_back_offset_m = h["shoulder_back_offset_m"].as<double>();
            cfg.human.upper_arm_length_m = h["upper_arm_length_m"].as<double>();
            cfg.human.forearm_length_m = h["forearm_length_m"].as<double>();
            cfg.human.neutral_swivel_right_rad = h["neutral_swivel_right_rad"].as<double>();
            cfg.human.neutral_swivel_left_rad = h["neutral_swivel_left_rad"].as<double>();
        }
        if (node["swivel_prior"]) {
            auto s = node["swivel_prior"];
            cfg.swivel.enabled = s["enabled"].as<bool>();
            cfg.swivel.neutral_gain = s["neutral_gain"].as<double>();
            cfg.swivel.continuity_gain = s["continuity_gain"].as<double>();
            cfg.swivel.wrist_orientation_gain = s["wrist_orientation_gain"].as<double>();
            cfg.swivel.max_swivel_rate_rad_s = s["max_swivel_rate_rad_s"].as<double>();
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

    std::string follower_ip = "172.30.21.146";
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
        else if (arg == "--dry-run") dry_run = true;
        else if (arg == "--vr-mapping-test") mapping_test = true;
        else if (arg == "--pose-ik") { /* handled after load_config */ }
        else if (arg == "--position-ik") { /* handled after load_config */ }
    }

    VrTeleopConfig cfg = load_config(config_path);
    if (ik_debug) cfg.debug.ik_debug = true; // Override by CLI
    
    // Command line overrides for IK mode
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--pose-ik") cfg.ik.mode = "pose";
        else if (arg == "--position-ik") cfg.ik.mode = "position";
    }

    LOG_INFO("Starting VR Bimanual Leader");
    LOG_INFO("Follower IP: " << follower_ip << " (Ports: " << right_port << ", " << left_port << ")");
    LOG_INFO("VR Port: " << vr_port << " | Telemetry Port: " << telemetry_port);

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
    std::mutex vr_mutex;
    VrRelativeTeleopPacketV1 last_vr_pkt;
    std::memset(&last_vr_pkt, 0, sizeof(last_vr_pkt));
    bool vr_connected = false;
    std::chrono::steady_clock::time_point last_vr_time;

    net::UdpReceiver vr_receiver("0.0.0.0", vr_port);
    vr_receiver.start_raw([&](const void* data, size_t size) {
        if (size < 8) {
            static auto last_size_err = std::chrono::steady_clock::now();
            if (std::chrono::steady_clock::now() - last_size_err > std::chrono::seconds(1)) {
                LOG_WARN("VR Packet too small: " << size << " bytes");
                last_size_err = std::chrono::steady_clock::now();
            }
            return;
        }

        uint32_t magic = *static_cast<const uint32_t*>(data);
        if (magic != VrRelativeTeleopPacketV1::MAGIC) {
            static auto last_magic_err = std::chrono::steady_clock::now();
            if (std::chrono::steady_clock::now() - last_magic_err > std::chrono::seconds(1)) {
                LOG_WARN("VR Invalid Magic: 0x" << std::hex << magic << " (Expected 0x" << VrRelativeTeleopPacketV1::MAGIC << ") size=" << std::dec << size);
                last_magic_err = std::chrono::steady_clock::now();
            }
            return;
        }

        if (size != sizeof(VrRelativeTeleopPacketV1)) {
            static auto last_len_err = std::chrono::steady_clock::now();
            if (std::chrono::steady_clock::now() - last_len_err > std::chrono::seconds(5)) {
                LOG_WARN("VR Packet size mismatch (Ignored): expected=" << sizeof(VrRelativeTeleopPacketV1) << " received=" << size);
                last_len_err = std::chrono::steady_clock::now();
            }
            return;
        }

        {
            std::lock_guard<std::mutex> lock(vr_mutex);
            std::memcpy(&last_vr_pkt, data, sizeof(VrRelativeTeleopPacketV1));
            vr_connected = true;
            last_vr_time = std::chrono::steady_clock::now();
        }
            
        // Raw debug: if any grip is pressed, show it
        if (last_vr_pkt.left_grip || last_vr_pkt.right_grip) {
            static int dbg_count = 0;
            if (dbg_count++ % 100 == 0) {
                std::cout << "[UDP RECV] L_grip=" << (int)last_vr_pkt.left_grip 
                          << " R_grip=" << (int)last_vr_pkt.right_grip 
                          << " R_delta_x=" << last_vr_pkt.right_delta_pos_robot[0] << std::endl;
            }
        }
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
                // Observation state: [R_arm(7), R_grip(1), L_arm(7), L_grip(1)]
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
        VrRelativeTeleopPacketV1 vr_pkt;
        {
            std::lock_guard<std::mutex> lock(vr_mutex);
            vr_pkt = last_vr_pkt;
        }

        std::array<double, 7> fb_r, fb_l;
        bool telemetry_stale = false;
        {
            std::lock_guard<std::mutex> lock(tel_mutex);
            fb_r = robot_q_r_fb;
            fb_l = robot_q_l_fb;
            
            auto now = std::chrono::steady_clock::now();
            auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tel_time).count();
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

        auto process_arm_logic = [&](net::ArmSide side, const VrRelativeTeleopPacketV1& pkt,
                                    OpenArmPinocchioIK& ik, ArmAnchor& anchor, control::TemporalSwivelEstimator& swivel_est,
                                    std::array<double, 7>& q_target, 
                                    double& grip_target, const std::array<double, 7>& q_feedback, bool& ik_success, 
                                    double& burst_timer) {
            
            if (telemetry_stale) {
                ik_success = false;
                return; 
            }

            bool grip_held = (side == net::ArmSide::RIGHT) ? pkt.right_grip : pkt.left_grip;
            bool recenter = (side == net::ArmSide::RIGHT) ? pkt.right_recenter_event : pkt.left_recenter_event;
            uint8_t trigger = (side == net::ArmSide::RIGHT) ? pkt.right_trigger : pkt.left_trigger;
            const float* delta_vr_robot = (side == net::ArmSide::RIGHT) ? pkt.right_delta_pos_robot : pkt.left_delta_pos_robot;
            
            grip_target = static_cast<double>(trigger) / 255.0;

            if (grip_held && tel_connected) {
                if (!anchor.active || recenter) {
                    ik.compute_fk_pose(q_feedback, anchor.robot_ee_pos_anchor, anchor.robot_ee_quat_anchor); 
                    anchor.robot_q_anchor = q_feedback;
                    anchor.active = true;
                    LOG_INFO((side==net::ArmSide::RIGHT?"RIGHT":"LEFT") << " Anchor updated. EE Pos: " << std::fixed << std::setprecision(3) 
                             << anchor.robot_ee_pos_anchor[0] << "," << anchor.robot_ee_pos_anchor[1] << "," << anchor.robot_ee_pos_anchor[2]);
                    burst_timer = cfg.debug.ik_debug_burst_sec;
                }

                // 1. Get Delta from Packet (already robot-space from Quest)
                Eigen::Vector3d delta_pos_in(pkt.right_delta_pos_robot[0], pkt.right_delta_pos_robot[1], pkt.right_delta_pos_robot[2]);
                if (side == net::ArmSide::LEFT) {
                    delta_pos_in = Eigen::Vector3d(pkt.left_delta_pos_robot[0], pkt.left_delta_pos_robot[1], pkt.left_delta_pos_robot[2]);
                }
                
                // 2. Mapping
                Eigen::Vector3d scale(cfg.mapping.position_scale_xyz[0], cfg.mapping.position_scale_xyz[1], cfg.mapping.position_scale_xyz[2]);
                Eigen::Matrix3d axis_mat = (side == net::ArmSide::RIGHT) ? cfg.mapping.right_axis_matrix : cfg.mapping.left_axis_matrix;
                Eigen::Vector3d delta_pos_robot = axis_mat * (scale.array() * delta_pos_in.array()).matrix();

                // Clamp delta pos to avoid runaway
                double d_pos_norm = delta_pos_robot.norm();
                if (d_pos_norm > cfg.mapping.max_delta_m) {
                    delta_pos_robot *= (cfg.mapping.max_delta_m / d_pos_norm);
                }
                
                const float* d_rot = (side == net::ArmSide::RIGHT) ? pkt.right_delta_rot_robot : pkt.left_delta_rot_robot;
                Eigen::Quaterniond delta_rot_vr(d_rot[3], d_rot[0], d_rot[1], d_rot[2]);
                Eigen::Matrix3d rot_map = (side == net::ArmSide::RIGHT) ? cfg.mapping.right_rotation_matrix : cfg.mapping.left_rotation_matrix;
                Eigen::Quaterniond delta_rot_robot(rot_map * delta_rot_vr.toRotationMatrix() * rot_map.transpose());

                std::array<double, 3> x_des = {
                    anchor.robot_ee_pos_anchor[0] + delta_pos_robot.x(),
                    anchor.robot_ee_pos_anchor[1] + delta_pos_robot.y(),
                    anchor.robot_ee_pos_anchor[2] + delta_pos_robot.z()
                };

                Eigen::Quaterniond q_anchor(anchor.robot_ee_quat_anchor[3], anchor.robot_ee_quat_anchor[0], anchor.robot_ee_quat_anchor[1], anchor.robot_ee_quat_anchor[2]);
                // LOCAL Multiplication: q_anchor * delta
                Eigen::Quaterniond quat_des = q_anchor * delta_rot_robot;

                // 3. Swivel Estimation (Phase 7)
                double swivel_target = 0.0;
                if (cfg.human.enabled) {
                    Eigen::Vector3d hmd_p(pkt.hmd_pos[0], pkt.hmd_pos[1], pkt.hmd_pos[2]);
                    Eigen::Quaterniond hmd_q(pkt.hmd_quat[3], pkt.hmd_quat[0], pkt.hmd_quat[1], pkt.hmd_quat[2]);
                    
                    double w = cfg.human.shoulder_width_m * (side == net::ArmSide::RIGHT ? 0.5 : -0.5);
                    Eigen::Vector3d shoulder_local(w, -cfg.human.shoulder_down_offset_m, -cfg.human.shoulder_back_offset_m);
                    Eigen::Vector3d shoulder_pos_vr = hmd_p + hmd_q * shoulder_local;
                    
                    Eigen::Vector3d wrist_pos_robot(x_des[0], x_des[1], x_des[2]);
                    
                    // Temporary: use a fixed shoulder position relative to robot base for now
                    Eigen::Vector3d shoulder_pos_robot(0.0, (side == net::ArmSide::RIGHT ? -0.2 : 0.2), 0.5); 
                    
                    control::SwivelInput s_in;
                    s_in.shoulder_pos = shoulder_pos_robot;
                    s_in.wrist_pos = wrist_pos_robot;
                    s_in.wrist_orientation = delta_rot_robot; 
                    s_in.dt = 1.0 / rate_hz;
                    s_in.right_arm = (side == net::ArmSide::RIGHT);
                    
                    swivel_target = swivel_est.update(s_in);
                }

                if (mapping_test) {
                    static int mt_cnt = 0;
                    if (mt_cnt++ % 50 == 0) {
                        std::cout << "[MAP TEST] " << (side==net::ArmSide::RIGHT?"R":"L") 
                                  << " Delta_pos: " << delta_pos_robot.x() << "," << delta_pos_robot.y() << "," << delta_pos_robot.z()
                                  << " Swivel: " << swivel_target << std::endl;
                    }
                    return;
                }

                std::array<double, 4> quat_des_arr = { quat_des.x(), quat_des.y(), quat_des.z(), quat_des.w() };

                std::array<double, 7> q_anchor_with_swivel = anchor.robot_q_anchor;
                // Heuristic: Joint 3 (index 2) typically controls the swivel rotation for this 7-DOF kinematic structure.
                // We offset it by the estimated swivel angle to bias the IK towards a natural posture.
                if (cfg.human.enabled) {
                    q_anchor_with_swivel[2] += swivel_target; 
                }

                std::array<double, 7> q_ik_out;
                bool solved = false;
                double residual_pos = 0, residual_ori = 0;
                if (cfg.ik.mode == "pose") {
                    solved = ik.solve_pose(x_des, quat_des_arr, q_feedback, q_ik_out, &q_anchor_with_swivel);
                } else {
                    solved = ik.solve(x_des, q_feedback, q_ik_out, &q_anchor_with_swivel);
                }

                if (!solved && cfg.debug.ik_debug) {
                    // Quick FK to see how far we are
                    std::array<double, 3> actual_pos;
                    std::array<double, 4> actual_quat;
                    ik.compute_fk_pose(q_ik_out, actual_pos, actual_quat);
                    residual_pos = std::sqrt(std::pow(actual_pos[0]-x_des[0],2) + std::pow(actual_pos[1]-x_des[1],2) + std::pow(actual_pos[2]-x_des[2],2));
                }

                if (solved) {
                    
                    const auto& max_steps = (side == net::ArmSide::RIGHT) ? cfg.rate_limit.max_step_rad_r : cfg.rate_limit.max_step_rad_l;
                    
                    std::array<double, 7> q_limited;
                    for (int i = 0; i < 7; ++i) {
                        double diff = q_ik_out[i] - q_target[i];
                        if (std::abs(diff) > max_steps[i]) {
                            diff = (diff > 0 ? 1.0 : -1.0) * max_steps[i];
                        }
                        q_limited[i] = q_target[i] + diff;
                    }

                    q_target = q_limited;
                    ik_success = true;
                }
                // Task 1 Logging
                if (cfg.debug.ik_debug) {
                    bool should_log = (burst_timer > 0);
                    if (!should_log) {
                        static auto last_t = std::chrono::steady_clock::now();
                        auto now = std::chrono::steady_clock::now();
                        if (std::chrono::duration<double>(now - last_t).count() > (1.0/cfg.debug.ik_debug_rate_hz)) {
                            should_log = true;
                            if (side == net::ArmSide::LEFT) last_t = now;
                        }
                    }
                    if (should_log) {
                        std::stringstream ss;
                        ss << "[IK DBG] " << (side==net::ArmSide::RIGHT?"R":"L") 
                           << " Success: " << (solved ? "YES" : "NO ") 
                           << " Err: " << std::fixed << std::setprecision(4) << residual_pos << "m "
                           << " TargetPos: " << x_des[0] << "," << x_des[1] << "," << x_des[2];
                        LOG_INFO(ss.str());
                    }
                    if (burst_timer > 0) burst_timer -= (1.0 / rate_hz);
                }
            } else {
                anchor.active = false;
                if (tel_connected) q_target = q_feedback;
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

        send_pkt(sender_r, net::ArmSide::RIGHT, target_q_r, target_grip_r, vr_pkt.right_grip);
        send_pkt(sender_l, net::ArmSide::LEFT, target_q_l, target_grip_l, vr_pkt.left_grip);

        // Logging (1Hz)
        auto now = std::chrono::steady_clock::now();
        if (now - last_log_time > std::chrono::seconds(1)) {
            std::cout << "\n--- VR Teleop Status ---" << std::endl;
            std::cout << "VR:  " << (vr_connected ? "OK" : "NO (Waiting for Quest2...)") << std::endl;
            std::cout << "TEL: " << (tel_connected ? (telemetry_stale ? "STALE (Timeout!)" : "OK") : "NO (Waiting for Follower --publish-telemetry ...)") << std::endl;
            
            auto log_arm = [&](const char* label, bool grip, const float delta[3], const std::array<double, 7>& q_fb, const std::array<double, 7>& q_target, bool ik_ok) {
                std::cout << label << ": " << (grip ? "[GRIP] " : "[IDLE] ");
                std::cout << "Delta: (" << std::fixed << std::setprecision(3) << delta[0] << "," << delta[1] << "," << delta[2] << ") ";
                std::cout << "IK: " << (ik_ok ? "OK" : "FAIL") << std::endl;
                std::cout << "  FB (j1-3): " << q_fb[0] << ", " << q_fb[1] << ", " << q_fb[2] << std::endl;
                std::cout << "  TG (j1-3): " << q_target[0] << ", " << q_target[1] << ", " << q_target[2] << std::endl;
            };

            log_arm("RIGHT", vr_pkt.right_grip, vr_pkt.right_delta_pos_robot, fb_r, target_q_r, right_ik_success);
            log_arm("LEFT ", vr_pkt.left_grip, vr_pkt.left_delta_pos_robot, fb_l, target_q_l, left_ik_success);
            
            last_log_time = now;
        }

        // Forward VR packet to visualization bridge
        viz_sender.send_raw(&vr_pkt, sizeof(vr_pkt));

        seq++;
        next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(period);
        std::this_thread::sleep_until(next_time);
    }

    vr_receiver.stop();
    tel_receiver.stop();
    return 0;
}
