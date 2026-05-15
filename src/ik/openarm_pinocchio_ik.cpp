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
    std::array<double, 4> dummy_quat;
    return compute_fk_pose(q_in, out_pos, dummy_quat);
}

bool OpenArmPinocchioIK::compute_fk_pose(const std::array<double, 7>& q_in, 
                                         std::array<double, 3>& out_pos,
                                         std::array<double, 4>& out_quat) {
    Eigen::VectorXd q = pinocchio::neutral(model_);
    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        int jid = joint_ids_[i];
        int q_idx = model_.joints[jid].idx_q();
        q[q_idx] = q_in[i];
    }

    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);
    
    const pinocchio::SE3& oMf = data_->oMf[ee_id_];
    Eigen::Vector3d pos = oMf.translation();
    Eigen::Quaterniond quat(oMf.rotation());
    
    out_pos[0] = pos[0];
    out_pos[1] = pos[1];
    out_pos[2] = pos[2];
    
    out_quat[0] = quat.x();
    out_quat[1] = quat.y();
    out_quat[2] = quat.z();
    out_quat[3] = quat.w();
    return true;
}

bool OpenArmPinocchioIK::solve(const std::array<double, 3>& target_pos,
                               const std::array<double, 7>& q_seed,
                               std::array<double, 7>& out_q,
                               const std::array<double, 7>* q_anchor) {
    // For backwards compatibility, use solve_pose with a dummy orientation and low weight
    std::array<double, 4> dummy_quat = {0, 0, 0, 1};
    double old_ori_weight = params_.orientation_weight;
    params_.orientation_weight = 0.0; // Ignore orientation
    bool res = solve_pose(target_pos, dummy_quat, q_seed, out_q, q_anchor);
    params_.orientation_weight = old_ori_weight;
    return res;
}

bool OpenArmPinocchioIK::solve_pose(const std::array<double, 3>& target_pos,
                                    const std::array<double, 4>& target_quat_xyzw,
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

    pinocchio::SE3 oMdes(
        Eigen::Quaterniond(target_quat_xyzw[3], target_quat_xyzw[0], target_quat_xyzw[1], target_quat_xyzw[2]).toRotationMatrix(),
        Eigen::Vector3d(target_pos[0], target_pos[1], target_pos[2])
    );

    bool success = false;
    double final_err_pos = 0;
    double final_err_ori = 0;

    for (int iter = 0; iter < params_.max_iterations; ++iter) {
        pinocchio::forwardKinematics(model_, *data_, q);
        pinocchio::updateFramePlacements(model_, *data_);
        
        const pinocchio::SE3& oMcurr = data_->oMf[ee_id_];
        pinocchio::SE3 dMi = oMcurr.actInv(oMdes);
        Eigen::Matrix<double, 6, 1> err = pinocchio::log6(dMi).toVector();
        // err is in LOCAL frame. 
        
        final_err_pos = err.head<3>().norm();
        final_err_ori = err.tail<3>().norm();
        
        if (final_err_pos < params_.convergence_tol_m && final_err_ori < params_.convergence_tol_rad) {
            success = true;
            break;
        }

        // Clamp task steps
        if (err.head<3>().norm() > params_.max_task_step_m) {
            err.head<3>() = err.head<3>().normalized() * params_.max_task_step_m;
        }
        if (err.tail<3>().norm() > params_.max_task_step_rad) {
            err.tail<3>() = err.tail<3>().normalized() * params_.max_task_step_rad;
        }

        // Apply weights
        err.head<3>() *= params_.position_weight;
        err.tail<3>() *= params_.orientation_weight;

        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model_.nv);
        J.setZero();
        pinocchio::computeFrameJacobian(model_, *data_, q, ee_id_, pinocchio::ReferenceFrame::LOCAL, J);
        
        // Damped Pseudo-inverse: dq = J^T * (J*J^T + lambda^2 * I)^-1 * err
        Eigen::Matrix<double, 6, 6> JJT = J * J.transpose();
        JJT.diagonal().array() += std::pow(params_.damping, 2);
        
        Eigen::Matrix<double, 6, 1> rhs = JJT.ldlt().solve(err);
        Eigen::VectorXd dq = J.transpose() * rhs;

        // Null-space posture stabilization
        if (params_.nullspace_enabled && q_anchor) {
            Eigen::VectorXd q_a = pinocchio::neutral(model_);
            for(size_t i=0; i<joint_ids_.size(); i++) {
                q_a[model_.joints[joint_ids_[i]].idx_q()] = (*q_anchor)[i];
            }

            Eigen::VectorXd post_err = pinocchio::difference(model_, q, q_a);
            
            // Project into null-space: dq_ns = (I - J# J) * gain * post_err
            Eigen::MatrixXd J_pinv = J.transpose() * JJT.inverse();
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
            Eigen::MatrixXd N = I - J_pinv * J;
            
            dq += N * (params_.nullspace_posture_gain * post_err);
        }

        if (dq.norm() > params_.max_dq_norm) {
            dq = dq.normalized() * params_.max_dq_norm;
        }

        q = pinocchio::integrate(model_, q, dq);

        for (int i = 0; i < model_.nq; ++i) {
            q[i] = std::max(model_.lowerPositionLimit[i], std::min(model_.upperPositionLimit[i], q[i]));
        }
    }

    for (size_t i = 0; i < joint_ids_.size(); ++i) {
        int jid = joint_ids_[i];
        int q_idx = model_.joints[jid].idx_q();
        out_q[i] = q[q_idx];
    }

    return success || (final_err_pos < params_.convergence_tol_m * 5.0);
}

} // namespace openarm_teleop
