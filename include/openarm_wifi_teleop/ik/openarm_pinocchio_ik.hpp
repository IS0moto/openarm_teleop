#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <string>
#include <vector>
#include <array>

// Disable FCL support to avoid version mismatch errors in hpp-fcl
#ifdef PINOCCHIO_WITH_HPP_FCL
#undef PINOCCHIO_WITH_HPP_FCL
#endif

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace openarm_teleop {

/**
 * @brief Pinocchio-based IK solver for OpenArm.
 */
class OpenArmPinocchioIK {
public:
    OpenArmPinocchioIK(const std::string& urdf_path, 
                       const std::string& ee_link_name,
                       const std::vector<std::string>& joint_names);
    ~OpenArmPinocchioIK();

    struct IKParams {
        int max_iterations = 20;
        double convergence_tol_m = 0.005;
        double convergence_tol_rad = 0.05;
        double damping = 0.05;
        double max_dq_norm = 0.05;
        double max_task_step_m = 0.02;
        double max_task_step_rad = 0.1;
        bool nullspace_enabled = true;
        double nullspace_posture_gain = 0.05;
        double position_weight = 1.0;
        double orientation_weight = 0.1;
    };

    void set_params(const IKParams& params) { params_ = params; }

    /**
     * @brief Solve IK for a target position (3D) using iterative DLS.
     */
    bool solve(const std::array<double, 3>& target_pos,
               const std::array<double, 7>& q_seed,
               std::array<double, 7>& out_q,
               const std::array<double, 7>* q_anchor = nullptr);

    /**
     * @brief Solve IK for a target pose (6D) using iterative DLS.
     * 
     * @param target_pos    Target position in robot base frame.
     * @param target_quat_xyzw Target orientation as quaternion (x,y,z,w).
     * @param q_seed        Initial joint angles.
     * @param out_q         Calculated joint angles.
     * @param q_anchor      Optional anchor posture.
     * @returns true if successful.
     */
    bool solve_pose(const std::array<double, 3>& target_pos,
                    const std::array<double, 4>& target_quat_xyzw,
                    const std::array<double, 7>& q_seed,
                    std::array<double, 7>& out_q,
                    const std::array<double, 7>* q_anchor = nullptr);

    /**
     * @brief Compute Forward Kinematics (3D position only).
     */
    bool compute_fk(const std::array<double, 7>& q_in, std::array<double, 3>& out_pos);

    /**
     * @brief Compute Forward Kinematics (6D pose).
     * 
     * @param q_in     Joint angles.
     * @param out_pos  Output position.
     * @param out_quat Output quaternion (x,y,z,w).
     * @returns true if successful.
     */
    bool compute_fk_pose(const std::array<double, 7>& q_in, 
                         std::array<double, 3>& out_pos,
                         std::array<double, 4>& out_quat);

private:
    IKParams params_;
    pinocchio::Model model_;
    pinocchio::Data* data_;
    int ee_id_;
    std::vector<int> joint_ids_;
    int njoints_model_;

    // IK parameters
    double damp_ = 1e-6;
    double max_step_ = 0.1; // max radians per step
};

} // namespace openarm_teleop
