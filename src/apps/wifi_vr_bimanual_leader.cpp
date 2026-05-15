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

using namespace openarm_wifi_teleop;
using namespace openarm_teleop;

std::atomic<bool> keep_running(true);
void signal_handler(int) { keep_running = false; }

struct ArmAnchor {
    bool active = false;
    std::array<double, 3> robot_ee_pos_anchor = {0, 0, 0};
    std::array<double, 7> robot_q_anchor = {0,0,0,0,0,0,0};
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);

    std::string follower_ip = "172.30.21.146";
    uint16_t right_port = 50000;
    uint16_t left_port = 50001;
    uint16_t vr_port = 54000;
    uint16_t telemetry_port = 51000;
    double rate_hz = 100.0; // IK loop rate
    std::string right_urdf = "urdf/openarm_right.urdf";
    std::string left_urdf = "urdf/openarm_left.urdf";
    std::string right_ee = "openarm_right_hand";
    std::string left_ee = "openarm_left_hand";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--follower-ip" && i + 1 < argc) follower_ip = argv[++i];
        else if (arg == "--vr-port" && i + 1 < argc) vr_port = std::stoi(argv[++i]);
        else if (arg == "--telemetry-port" && i + 1 < argc) telemetry_port = std::stoi(argv[++i]);
        else if (arg == "--rate-hz" && i + 1 < argc) rate_hz = std::stod(argv[++i]);
        else if (arg == "--right-urdf" && i + 1 < argc) right_urdf = argv[++i];
        else if (arg == "--left-urdf" && i + 1 < argc) left_urdf = argv[++i];
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

    // VR Data
    std::mutex vr_mutex;
    VrRelativeTeleopPacketV1 last_vr_pkt;
    std::memset(&last_vr_pkt, 0, sizeof(last_vr_pkt));
    bool vr_connected = false;

    net::UdpReceiver vr_receiver("0.0.0.0", vr_port);
    vr_receiver.start_raw([&](const void* data, size_t size) {
        if (size >= 8) {
            uint32_t magic = *static_cast<const uint32_t*>(data);
            if (magic != VrRelativeTeleopPacketV1::MAGIC) {
                // Occasional print
                static auto last_magic_err = std::chrono::steady_clock::now();
                auto now = std::chrono::steady_clock::now();
                if (now - last_magic_err > std::chrono::seconds(1)) {
                    std::cout << "[ERROR] Invalid Magic: 0x" << std::hex << magic << " expected 0x" << VrRelativeTeleopPacketV1::MAGIC << std::endl;
                    last_magic_err = now;
                }
            }
        }
        if (size == sizeof(VrRelativeTeleopPacketV1)) {
            std::lock_guard<std::mutex> lock(vr_mutex);
            std::memcpy(&last_vr_pkt, data, sizeof(VrRelativeTeleopPacketV1));
            vr_connected = true;
            
            // Raw debug: if any grip is pressed, show it
            if (last_vr_pkt.left_grip || last_vr_pkt.right_grip) {
                static int dbg_count = 0;
                if (dbg_count++ % 100 == 0) {
                    std::cout << "[UDP RECV] L_grip=" << (int)last_vr_pkt.left_grip 
                              << " R_grip=" << (int)last_vr_pkt.right_grip 
                              << " R_delta_x=" << last_vr_pkt.right_delta_pos_robot[0] << std::endl;
                }
            }
        }
    });

    // Telemetry Data (Robot Feedback for Sync)
    std::mutex tel_mutex;
    std::array<double, 7> robot_q_r_fb = {0,0,0,0,0,0,0};
    std::array<double, 7> robot_q_l_fb = {0,0,0,0,0,0,0};
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

    LOG_INFO("Entering control loop at " << rate_hz << " Hz");

    while (keep_running) {
        VrRelativeTeleopPacketV1 vr_pkt;
        {
            std::lock_guard<std::mutex> lock(vr_mutex);
            vr_pkt = last_vr_pkt;
        }

        bool right_ik_success = false;
        bool left_ik_success = false;

        auto process_arm = [&](bool grip_held, bool recenter, const float delta_robot[3], uint8_t trigger,
                               OpenArmPinocchioIK& ik, ArmAnchor& anchor, std::array<double, 7>& q_target, 
                               double& grip_target, const std::array<double, 7>& q_feedback, bool& ik_success, int& burst_log_counter) {
            grip_target = static_cast<double>(trigger) / 255.0;

            if (grip_held && tel_connected) {
                if (!anchor.active || recenter) {
                    ik.compute_fk(q_target, anchor.robot_ee_pos_anchor);
                    anchor.robot_q_anchor = q_target;
                    anchor.active = true;
                    LOG_INFO("Anchor updated: " << std::fixed << std::setprecision(3) 
                             << anchor.robot_ee_pos_anchor[0] << ", " << anchor.robot_ee_pos_anchor[1] << ", " << anchor.robot_ee_pos_anchor[2]);
                    burst_log_counter = 20; // Log next 20 frames
                }

                std::array<double, 3> x_des = {
                    anchor.robot_ee_pos_anchor[0] + delta_robot[0],
                    anchor.robot_ee_pos_anchor[1] + delta_robot[1],
                    anchor.robot_ee_pos_anchor[2] + delta_robot[2]
                };

                std::array<double, 7> next_q;
                if (ik.solve(x_des, q_target, next_q)) {
                    // SAFETY: Limit max joint change per frame in Leader
                    bool jump_detected = false;
                    for (int i = 0; i < 7; ++i) {
                        if (std::abs(next_q[i] - q_target[i]) > 0.3) { // ~17 degrees per frame is huge
                            jump_detected = true;
                        }
                    }

                    if (!jump_detected) {
                        q_target = next_q;
                        ik_success = true;
                    } else {
                        LOG_WARN("IK JUMP REJECTED!");
                    }
                }
            } else {
                anchor.active = false;
                if (tel_connected) {
                    q_target = q_feedback;
                }
            }

            if (burst_log_counter > 0) {
                std::cout << "[BURST] TG: ";
                for(int i=0; i<7; i++) std::cout << std::fixed << std::setprecision(3) << q_target[i] << (i==6?"":",");
                std::cout << " | FB: ";
                for(int i=0; i<7; i++) std::cout << q_feedback[i] << (i==6?"":",");
                std::cout << std::endl;
                burst_log_counter--;
            }
        };

        std::array<double, 7> fb_r, fb_l;
        {
            std::lock_guard<std::mutex> lock(tel_mutex);
            fb_r = robot_q_r_fb;
            fb_l = robot_q_l_fb;
        }

        static int burst_r = 0, burst_l = 0;
        process_arm(vr_pkt.right_grip, vr_pkt.right_recenter_event, vr_pkt.right_delta_pos_robot, vr_pkt.right_trigger, ik_r, anchor_r, target_q_r, target_grip_r, fb_r, right_ik_success, burst_r);
        process_arm(vr_pkt.left_grip, vr_pkt.left_recenter_event, vr_pkt.left_delta_pos_robot, vr_pkt.left_trigger, ik_l, anchor_l, target_q_l, target_grip_l, fb_l, left_ik_success, burst_l);

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
            // ONLY enable if grip is held AND we have telemetry AND not in estop
            pkt.enable = (arm_enabled && tel_connected && !vr_pkt.estop) ? 1 : 0;
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
            std::cout << "VR: " << (vr_connected ? "OK" : "NO") << " | TEL: " << (tel_connected ? "OK" : "NO") << std::endl;
            
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

        seq++;
        next_time += std::chrono::duration_cast<std::chrono::nanoseconds>(period);
        std::this_thread::sleep_until(next_time);
    }

    vr_receiver.stop();
    tel_receiver.stop();
    return 0;
}
