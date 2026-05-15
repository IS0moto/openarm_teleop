#ifdef PINOCCHIO_WITH_HPP_FCL
#undef PINOCCHIO_WITH_HPP_FCL
#endif
#include <pinocchio/fwd.hpp>
#include "openarm_wifi_teleop/ik/openarm_pinocchio_ik.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <iostream>
#include <stdexcept>

namespace openarm_teleop {

OpenArmPinocchioIK::OpenArmPinocchioIK(const std::string& urdf_path, 
                                     const std::string& ee_link_name,
                                     const std::vector<std::string>& joint_names) {
    // Build model from URDF
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = new pinocchio::Data(model_);
    
    // Find End-Effector frame ID
    if (!model_.existFrame(ee_link_name)) {
        throw std::runtime_error("End-effector frame not found: " + ee_link_name);
    }
    ee_id_ = model_.getFrameId(ee_link_name);
    njoints_model_ = model_.nv;

    // Map joint names to joint IDs
    for (const auto& name : joint_names) {
        if (!model_.existJointName(name)) {
            throw std::runtime_error("Joint not found in model: " + name);
        }
        joint_ids_.push_back((int)model_.getJointId(name));
    }
}

OpenArmPinocchioIK::~OpenArmPinocchioIK() {
    delete data_;
}

bool OpenArmPinocchioIK::compute_fk(const std::array<double, 7>& q_in, std::array<double, 3>& out_pos) {
    Eigen::VectorXd q = pinocchio::neutral(model_);
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        int jid = joint_ids_[i];
        int q_idx = model_.joints[jid].idx_q();
        q[q_idx] = q_in[i];
    }

    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);
    Eigen::Vector3d pos = data_->oMf[ee_id_].translation();
    
    out_pos[0] = pos[0];
    out_pos[1] = pos[1];
    out_pos[2] = pos[2];
    return true;
}

bool OpenArmPinocchioIK::solve(const std::array<double, 3>& target_pos,
                               const std::array<double, 7>& current_q,
                               std::array<double, 7>& out_q) {
    // Current configuration in Eigen
    Eigen::VectorXd q = pinocchio::neutral(model_);
    for (size_t i = 0; i < (int)joint_ids_.size(); ++i) {
        int jid = joint_ids_[i];
        int q_idx = model_.joints[jid].idx_q();
        q[q_idx] = current_q[i];
    }

    // Target position
    Eigen::Vector3d x_des(target_pos[0], target_pos[1], target_pos[2]);

    // Compute current EE pose
    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);
    Eigen::Vector3d x_curr = data_->oMf[ee_id_].translation();

    // Position error
    Eigen::Vector3d err = x_des - x_curr;

    // Numerical check for NaN in error
    if (std::isnan(err.norm())) return false;

    // Compute Jacobian directly for the frame
    Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
    J.setZero();
    pinocchio::computeFrameJacobian(model_, *data_, q, ee_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    // Position part of Jacobian
    Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos = J.topRows<3>();

    // Use Damped Least Squares with more robust solver
    Eigen::Matrix3d JJT = J_pos * J_pos.transpose();
    
    // Ensure damp_ is not zero
    double d2 = (damp_ > 0) ? (damp_ * damp_) : 0.0001;
    JJT.diagonal().array() += d2;
    
    // Using COD for maximum stability even in singularity
    Eigen::Vector3d rhs = JJT.completeOrthogonalDecomposition().solve(err);
    Eigen::VectorXd dq_full = J_pos.transpose() * rhs;

    // Numerical debug (throttled)
    static int dbg_cnt = 0;
    if (dbg_cnt++ % 100 == 0 && err.norm() > 0.001) {
        std::cout << "[IK DEBUG] err_norm=" << err.norm() << " J_norm=" << J_pos.norm() << " JJT_det=" << JJT.determinant() << " dq_norm=" << dq_full.norm() << std::endl;
        if (dq_full.norm() < 1e-9) {
            std::cout << "  ! ALERT: dq is nearly zero despite error !" << std::endl;
        }
    }

    // Check for NaN in dq
    if (std::isnan(dq_full.norm())) return false;

    // Extract only the dq for our 7 joints
    Eigen::VectorXd dq_7 = Eigen::VectorXd::Zero(7);
    for (size_t i = 0; i < (int)joint_ids_.size(); ++i) {
        int v_idx = model_.joints[joint_ids_[i]].idx_v();
        dq_7[i] = dq_full[v_idx];
    }

    // Clamp step
    double norm = dq_7.norm();
    if (norm > max_step_) {
        dq_7 *= (max_step_ / norm);
    }

    // Update joint positions
    for (int i = 0; i < 7; ++i) {
        out_q[i] = current_q[i] + dq_7[i];
    }

    return true;
}

} // namespace openarm_teleop
