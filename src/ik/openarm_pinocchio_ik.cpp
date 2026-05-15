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
                               const std::array<double, 7>& q_seed,
                               std::array<double, 7>& out_q,
                               const std::array<double, 7>* q_anchor) {
    // Start with seed
    Eigen::VectorXd q = pinocchio::neutral(model_);
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        int jid = joint_ids_[i];
        int q_idx = model_.joints[jid].idx_q();
        q[q_idx] = q_seed[i];
    }

    Eigen::Vector3d x_des(target_pos[0], target_pos[1], target_pos[2]);
    bool success = false;
    double final_err = 0;

    for (int iter = 0; iter < params_.max_iterations; ++iter) {
        pinocchio::forwardKinematics(model_, *data_, q);
        pinocchio::updateFramePlacements(model_, *data_);
        
        Eigen::Vector3d x_curr = data_->oMf[ee_id_].translation();
        Eigen::Vector3d err = x_des - x_curr;
        
        final_err = err.norm();
        if (final_err < params_.convergence_tol_m) {
            success = true;
            break;
        }

        // Clamp task step to prevent large jumps in a single iteration
        if (err.norm() > params_.max_task_step_m) {
            err = err.normalized() * params_.max_task_step_m;
        }

        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
        J.setZero();
        pinocchio::computeFrameJacobian(model_, *data_, q, ee_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
        
        Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos = J.topRows<3>();
        
        // Damped Pseudo-inverse: dq = J^T * (J*J^T + lambda^2 * I)^-1 * err
        Eigen::Matrix3d JJT = J_pos * J_pos.transpose();
        JJT.diagonal().array() += std::pow(params_.damping, 2);
        
        Eigen::Vector3d rhs = JJT.ldlt().solve(err);
        Eigen::VectorXd dq = J_pos.transpose() * rhs;

        // Task 7: Null-space posture stabilization
        if (params_.nullspace_enabled && q_anchor) {
            Eigen::VectorXd q_a = pinocchio::neutral(model_);
            for(size_t i=0; i<joint_ids_.size(); i++) {
                q_a[model_.joints[joint_ids_[i]].idx_q()] = (*q_anchor)[i];
            }

            // Posture error (in tangent space)
            Eigen::VectorXd post_err = pinocchio::difference(model_, q, q_a);
            
            // Project into null-space: dq_ns = (I - J# J) * gain * post_err
            Eigen::MatrixXd J_pinv = J_pos.transpose() * JJT.inverse();
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
            Eigen::MatrixXd N = I - J_pinv * J_pos;
            
            dq += N * (params_.nullspace_posture_gain * post_err);
        }

        // Clamp dq norm per iteration
        if (dq.norm() > params_.max_dq_norm) {
            dq = dq.normalized() * params_.max_dq_norm;
        }

        q = pinocchio::integrate(model_, q, dq);

        // Task 8: Joint limit handling
        for (int i = 0; i < model_.nq; ++i) {
            q[i] = std::max(model_.lowerPositionLimit[i], std::min(model_.upperPositionLimit[i], q[i]));
        }
    }

    // Extract back to array
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        int jid = joint_ids_[i];
        int q_idx = model_.joints[jid].idx_q();
        out_q[i] = q[q_idx];
    }

    // Consider it "successful" if error is within a reasonable tolerance
    return success || (final_err < params_.convergence_tol_m * 5.0);
}

} // namespace openarm_teleop
