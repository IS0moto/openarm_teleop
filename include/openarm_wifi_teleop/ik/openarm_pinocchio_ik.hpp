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

    /**
     * @brief Solve IK for a target position.
     * 
     * @param target_pos  Target position in robot base frame.
     * @param current_q   Current joint angles (7 DOF).
     * @param out_q       Calculated next joint angles.
     * @returns true if successful.
     */
    bool solve(const std::array<double, 3>& target_pos,
               const std::array<double, 7>& current_q,
               std::array<double, 7>& out_q);

    /**
     * @brief Compute Forward Kinematics for current joints.
     * 
     * @param q_in    Joint angles (7 DOF).
     * @param out_pos End-effector position.
     * @returns true if successful.
     */
    bool compute_fk(const std::array<double, 7>& q_in, std::array<double, 3>& out_pos);

private:
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
