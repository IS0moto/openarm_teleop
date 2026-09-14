// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <iomanip>
#include <thread>

Control::Control(openarm::can::socket::OpenArm* arm, Dynamics* dynamics_l, Dynamics* dynamics_f,
                 std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
                 size_t arm_motor_num, size_t hand_motor_num)
    : openarm_(arm),
      dynamics_l_(dynamics_l),
      dynamics_f_(dynamics_f),
      robot_state_(robot_state),
      Ts_(Ts),
      role_(role),
      arm_motor_num_(arm_motor_num),
      hand_motor_num_(hand_motor_num) {
    differentiator_ = new Differentiator(Ts);
    openarmjointconverter_ = new OpenArmJointConverter(arm_motor_num_);
    openarmgripperjointconverter_ = new OpenArmJGripperJointConverter(hand_motor_num_);
    SetEffortLimits({});
}

Control::Control(openarm::can::socket::OpenArm* arm, Dynamics* dynamics_l, Dynamics* dynamics_f,
                 std::shared_ptr<RobotSystemState> robot_state, double Ts, int role,
                 std::string arm_type, size_t arm_motor_num, size_t hand_motor_num)
    : openarm_(arm),
      dynamics_l_(dynamics_l),
      dynamics_f_(dynamics_f),
      robot_state_(robot_state),
      Ts_(Ts),
      role_(role),
      arm_motor_num_(arm_motor_num),
      hand_motor_num_(hand_motor_num) {
    differentiator_ = new Differentiator(Ts);
    openarmjointconverter_ = new OpenArmJointConverter(arm_motor_num_);
    openarmgripperjointconverter_ = new OpenArmJGripperJointConverter(hand_motor_num_);
    SetEffortLimits({});

    arm_type_ = arm_type;
}

Control::~Control() {
    std::cout << "Control destructed " << std::endl;
    delete openarmjointconverter_;
    delete differentiator_;
}

// bool Control::Setup(void)
// {
//         // double motor_position[NMOTORS] = {0.0};

//         // ComputeJointPosition(motor_position, response_->position.data());

//         std::cout << "!control->Setup()  finished "<< std::endl;

//         return true;
// }

void Control::Shutdown(void) {
    std::cout << "control shutdown !!!" << std::endl;

    openarm_->disable_all();
}

void Control::SetParameter(const std::vector<double>& Kp, const std::vector<double>& Kd,
                           const std::vector<double>& Fc, const std::vector<double>& k,
                           const std::vector<double>& Fv, const std::vector<double>& Fo) {
    Kp_ = Kp;
    Kd_ = Kd;
    Fc_ = Fc;
    k_ = k;
    Fv_ = Fv;
    Fo_ = Fo;
}

void Control::SetGravityTrim(const std::vector<double>& scale, const std::vector<double>& offset) {
    gscale_ = scale;
    goffset_ = offset;
}

void Control::SetEffortLimits(const std::vector<double>& limits) {
    // Defaults: effort_limit_L for the arm joints, 2 Nm for the gripper.
    effort_limit_.assign(effort_limit_L, effort_limit_L + NMOTORS);
    effort_limit_[NMOTORS - 1] = 2.0;
    for (size_t i = 0; i < limits.size() && i < effort_limit_.size(); ++i) {
        if (std::isfinite(limits[i]) && limits[i] > 0.0) effort_limit_[i] = limits[i];
    }
}

void Control::SetBilateralGains(const std::vector<double>& Kp, const std::vector<double>& Kd,
                                const std::vector<double>& Kf) {
    bkp_ = Kp;
    bkd_ = Kd;
    bkf_ = Kf;
}

void Control::SetFollowerGravityFeedforward(bool enabled, double scale) {
    follower_gravity_ff_ = enabled;
    follower_gravity_ff_scale_ = scale;
}

double Control::ClampLeaderEffort(size_t index, double tau) const {
    if (!std::isfinite(tau)) return 0.0;
    const double lim = (index < effort_limit_.size()) ? effort_limit_[index] : 0.0;
    return std::max(-lim, std::min(lim, tau));
}

void Control::ComputeAllFriction(const std::vector<double>& arm_velocity,
                                 const std::vector<double>& gripper_velocity,
                                 std::vector<double>& friction) {
    std::vector<double> velocity(arm_velocity.size() + gripper_velocity.size(), 0.0);
    std::copy(arm_velocity.begin(), arm_velocity.end(), velocity.begin());
    std::copy(gripper_velocity.begin(), gripper_velocity.end(),
              velocity.begin() + arm_velocity.size());
    friction.assign(velocity.size(), 0.0);
    for (size_t i = 0; i < velocity.size(); ++i) ComputeFriction(velocity.data(), friction.data(), i);
}

bool Control::bilateral_step() {
    // get motor status
    std::vector<MotorState> arm_motor_states;
    const auto& arm_motors = openarm_->get_arm().get_motors();
    for (size_t i = 0; i < arm_motors.size(); ++i) {
        const auto& motor = arm_motors[i];
        arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }

    std::vector<MotorState> gripper_motor_states;
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    for (size_t i = 0; i < gripper_motors.size(); ++i) {
        const auto& motor = gripper_motors[i];
        gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }

    // convert joint to motor
    std::vector<JointState> joint_arm_states =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_gripper_states =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    // set reponse
    robot_state_->arm_state().set_all_responses(joint_arm_states);
    robot_state_->hand_state().set_all_responses(joint_gripper_states);

    size_t arm_dof = robot_state_->arm_state().get_size();
    size_t gripper_dof = robot_state_->hand_state().get_size();

    std::vector<double> joint_arm_positions(arm_dof, 0.0);
    std::vector<double> joint_arm_velocities(arm_dof, 0.0);
    std::vector<double> joint_arm_efforts(arm_dof, 0.0);

    std::vector<double> joint_gripper_positions(gripper_dof, 0.0);
    std::vector<double> joint_gripper_velocities(gripper_dof, 0.0);
    std::vector<double> joint_gripper_efforts(gripper_dof, 0.0);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions[i] = joint_arm_states[i].position;
        joint_arm_velocities[i] = joint_arm_states[i].velocity;
    }

    for (size_t i = 0; i < gripper_dof; ++i) {
        joint_gripper_positions[i] = joint_gripper_states[i].position;
        joint_gripper_velocities[i] = joint_gripper_states[i].velocity;
    }

    std::vector<double> gravity(arm_dof, 0.0);
    std::vector<double> coriolis(arm_dof, 0.0);
    std::vector<double> friction(arm_dof + gripper_dof, 0.0);

    std::vector<JointState> joint_arm_states_ref = robot_state_->arm_state().get_all_references();
    std::vector<JointState> joint_gripper_states_ref =
        robot_state_->hand_state().get_all_references();

    std::vector<double> joint_arm_positions_ref(arm_dof);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions_ref[i] = joint_arm_states_ref[i].position;
    }

    if (role_ == ROLE_LEADER) {
        dynamics_l_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_l_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(),
                                 coriolis.data());

    } else if (role_ == ROLE_FOLLOWER) {
        dynamics_f_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_f_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(),
                                 coriolis.data());
    }

    // Friction (compute joint friction)
    ComputeAllFriction(joint_arm_velocities, joint_gripper_velocities, friction);

    // set gravity and friciton comp joint torque value
    for (size_t i = 0; i < arm_dof; i++) {
        joint_arm_states_ref[i].effort = gravity[i] + friction[i];
    }

    for (size_t i = 0; i < gripper_dof; i++) {
        joint_gripper_states_ref[i].effort = friction[i + arm_dof];
    }

    std::vector<MotorState> motor_arm_states =
        openarmjointconverter_->joint_to_motor(joint_arm_states_ref);
    std::vector<MotorState> motor_gripper_states =
        openarmgripperjointconverter_->joint_to_motor(joint_gripper_states_ref);

    // kp kd q dq tau
    std::vector<openarm::damiao_motor::MITParam> arm_cmds;
    arm_cmds.reserve(arm_dof);
    for (size_t i = 0; i < arm_dof; ++i) {
        arm_cmds.emplace_back(openarm::damiao_motor::MITParam{
            Kp_[i], Kd_[i], motor_arm_states[i].position, motor_arm_states[i].velocity,
            motor_arm_states[i].effort});
    }

    // gripper command mit param
    std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
    gripper_cmds.reserve(gripper_dof);
    for (size_t i = 0; i < gripper_dof; ++i) {
        gripper_cmds.emplace_back(openarm::damiao_motor::MITParam{
            Kp_[i + arm_dof], Kd_[i + arm_dof], motor_gripper_states[i].position,
            motor_gripper_states[i].velocity, motor_gripper_states[i].effort});
    }

    // send command to arm
    openarm_->get_arm().mit_control_all(arm_cmds);
    // send command to gripper
    openarm_->get_gripper().mit_control_all(gripper_cmds);

    std::this_thread::sleep_for(std::chrono::microseconds(200));

    openarm_->recv_all(220);

    return true;
}

bool Control::unilateral_step() {
    // get motor status
    // get motor status (effort = measured motor torque, fed back to the leader
    // in bilateral mode)
    std::vector<MotorState> arm_motor_states;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }

    std::vector<MotorState> gripper_motor_states;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }

    // convert joint to motor
    std::vector<JointState> joint_arm_states =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_gripper_states =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    // set reponse
    robot_state_->arm_state().set_all_responses(joint_arm_states);
    robot_state_->hand_state().set_all_responses(joint_gripper_states);

    size_t arm_dof = robot_state_->arm_state().get_size();
    size_t gripper_dof = robot_state_->hand_state().get_size();

    std::vector<double> joint_arm_positions(arm_dof, 0.0);
    std::vector<double> joint_arm_velocities(arm_dof, 0.0);
    std::vector<double> joint_gripper_positions(gripper_dof, 0.0);
    std::vector<double> joint_gripper_velocities(gripper_dof, 0.0);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions[i] = joint_arm_states[i].position;
        joint_arm_velocities[i] = joint_arm_states[i].velocity;
    }

    for (size_t i = 0; i < gripper_dof; ++i) {
        joint_gripper_positions[i] = joint_gripper_states[i].position;
        joint_gripper_velocities[i] = joint_gripper_states[i].velocity;
    }

    std::vector<double> gravity(arm_dof, 0.0);
    std::vector<double> coriolis(arm_dof, 0.0);
    std::vector<double> friction(arm_dof + gripper_dof, 0.0);

    if (role_ == ROLE_LEADER) {
        return leader_step(nullptr);
    }

    else if (role_ == ROLE_FOLLOWER) {
        std::vector<JointState> joint_arm_states_ref =
            robot_state_->arm_state().get_all_references();
        std::vector<JointState> joint_hand_states_ref =
            robot_state_->hand_state().get_all_references();

        // Optional gravity feedforward (bilateral): the follower otherwise sags
        // by gravity/Kp, and that steady-state error is what the leader feels.
        std::vector<double> ff(arm_dof, 0.0);
        if (follower_gravity_ff_ && dynamics_f_) {
            dynamics_f_->GetGravity(joint_arm_positions.data(), gravity.data());
            for (size_t i = 0; i < arm_dof; ++i) {
                double g = gravity[i] * follower_gravity_ff_scale_;
                if (!std::isfinite(g)) g = 0.0;
                const double lim = (i < NMOTORS) ? effort_limit_F[i] : 0.0;
                ff[i] = std::max(-lim, std::min(lim, g));
            }
        }

        // Joint → Motor
        std::vector<MotorState> arm_motor_refs =
            openarmjointconverter_->joint_to_motor(joint_arm_states_ref);
        std::vector<MotorState> hand_motor_refs =
            openarmgripperjointconverter_->joint_to_motor(joint_hand_states_ref);

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_motor_refs.size());
        for (size_t i = 0; i < arm_motor_refs.size(); ++i) {
            arm_cmds.emplace_back(openarm::damiao_motor::MITParam{
                Kp_[i], Kd_[i], arm_motor_refs[i].position, arm_motor_refs[i].velocity,
                (i < ff.size()) ? ff[i] : 0.0});
        }

        std::vector<openarm::damiao_motor::MITParam> hand_cmds;
        hand_cmds.reserve(hand_motor_refs.size());
        for (size_t i = 0; i < hand_motor_refs.size(); ++i) {
            hand_cmds.emplace_back(openarm::damiao_motor::MITParam{
                Kp_[i + arm_dof], Kd_[i + arm_dof], hand_motor_refs[i].position,
                hand_motor_refs[i].velocity, 0.0});
        }

        openarm_->get_arm().mit_control_all(arm_cmds);
        openarm_->get_gripper().mit_control_all(hand_cmds);

        openarm_->recv_all(200);

        return true;
    }

    return true;
}

bool Control::leader_step(const LeaderBilateralCommand* cmd) {
    // get motor status (effort = measured motor torque)
    std::vector<MotorState> arm_motor_states;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }
    std::vector<MotorState> gripper_motor_states;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), motor.get_torque()});
    }

    std::vector<JointState> joint_arm_states =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_gripper_states =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    robot_state_->arm_state().set_all_responses(joint_arm_states);
    robot_state_->hand_state().set_all_responses(joint_gripper_states);

    size_t arm_dof = robot_state_->arm_state().get_size();
    size_t gripper_dof = robot_state_->hand_state().get_size();

    std::vector<double> joint_arm_positions(arm_dof, 0.0);
    std::vector<double> joint_arm_velocities(arm_dof, 0.0);
    std::vector<double> joint_gripper_positions(gripper_dof, 0.0);
    std::vector<double> joint_gripper_velocities(gripper_dof, 0.0);

    for (size_t i = 0; i < arm_dof; ++i) {
        joint_arm_positions[i] = joint_arm_states[i].position;
        joint_arm_velocities[i] = joint_arm_states[i].velocity;
    }
    for (size_t i = 0; i < gripper_dof; ++i) {
        joint_gripper_positions[i] = joint_gripper_states[i].position;
        joint_gripper_velocities[i] = joint_gripper_states[i].velocity;
    }

    std::vector<double> gravity(arm_dof, 0.0);
    std::vector<double> coriolis(arm_dof, 0.0);
    std::vector<double> friction(arm_dof + gripper_dof, 0.0);

    // Bilateral term is active only with a command, a positive ramp scale and
    // configured gains; everything else falls through to plain gravity comp.
    const bool bilateral = cmd && cmd->scale > 0.0 && bkp_.size() >= arm_dof + gripper_dof &&
                           bkd_.size() >= arm_dof + gripper_dof &&
                           cmd->arm_pos_ref.size() >= arm_dof &&
                           cmd->hand_pos_ref.size() >= gripper_dof;
    const double scale = bilateral ? std::max(0.0, std::min(1.0, cmd->scale)) : 0.0;

    // Model gravity at the follower pose, used to strip gravity from the
    // follower's measured torque so the force channel carries contact only.
    std::vector<double> gravity_ref(arm_dof, 0.0);
    const bool use_kf = bilateral && bkf_.size() >= arm_dof + gripper_dof &&
                        cmd->arm_tau_f.size() >= arm_dof;
    if (use_kf) {
        dynamics_l_->GetGravity(cmd->arm_pos_ref.data(), gravity_ref.data());
    }

    {
        // calc dynamics
        dynamics_l_->GetGravity(joint_arm_positions.data(), gravity.data());
        dynamics_l_->GetCoriolis(joint_arm_positions.data(), joint_arm_velocities.data(),
                                 coriolis.data());

        ComputeAllFriction(joint_arm_velocities, joint_gripper_velocities, friction);

        // arm joint state (leader gravity comp, with per-joint gravity trim)
        //   grav_cmd[i] = gravity[i]*gscale_[i] + goffset_[i]   (empty vectors => identity)
        std::vector<JointState> joint_arm_state_torque(arm_dof);
        std::vector<double> grav_cmd(arm_dof, 0.0);
        for (size_t i = 0; i < arm_dof; ++i) {
            const double gs = (i < gscale_.size()) ? gscale_[i] : 1.0;
            const double go = (i < goffset_.size()) ? goffset_[i] : 0.0;
            grav_cmd[i] = gravity[i] * gs + go;
            joint_arm_state_torque[i].position = joint_arm_positions[i];
            joint_arm_state_torque[i].velocity = joint_arm_velocities[i];
            double tau_b = 0.0;
            if (bilateral) {
                tau_b = bkp_[i] * (cmd->arm_pos_ref[i] - joint_arm_positions[i]) -
                        bkd_[i] * joint_arm_velocities[i];
                if (use_kf) tau_b -= bkf_[i] * (cmd->arm_tau_f[i] - gravity_ref[i]);
                tau_b *= scale;
            }
            joint_arm_state_torque[i].effort = ClampLeaderEffort(
                i, grav_cmd[i] + friction[i] * 0.3 + coriolis[i] * 0.1 + tau_b);
        }

        // gripper joint state
        std::vector<JointState> joint_gripper_state_torque(gripper_dof);
        for (size_t i = 0; i < gripper_dof; ++i) {
            joint_gripper_state_torque[i].position = joint_gripper_positions[i];
            joint_gripper_state_torque[i].velocity = joint_gripper_velocities[i];
            double tau_b = 0.0;
            if (bilateral) {
                // Gripper PD in MOTOR space: the joint<->motor map has a negative
                // gain (m -> rad) and effort passes through unconverted, so the
                // reference is mapped to motor position and the motor's own
                // position/velocity/torque are used. Gains are Nm/rad like the
                // follower's gripper Kp/Kd.
                std::vector<JointState> ref_joint(1);
                ref_joint[0].position = cmd->hand_pos_ref[i];
                const double motor_ref = openarmgripperjointconverter_->joint_to_motor(ref_joint)[0].position;
                tau_b = bkp_[arm_dof + i] * (motor_ref - gripper_motor_states[i].position) -
                        bkd_[arm_dof + i] * gripper_motor_states[i].velocity;
                if (use_kf && i < cmd->hand_tau_f.size()) tau_b -= bkf_[arm_dof + i] * cmd->hand_tau_f[i];
                tau_b *= scale;
            }
            joint_gripper_state_torque[i].effort =
                ClampLeaderEffort(arm_dof + i, friction[arm_dof + i] * 0.3 + tau_b);
        }

        // One-time trace of the first leader cycle: this is where a bad gripper
        // torque showed up once (out-of-bounds friction read, since fixed).
        if (!leader_first_cycle_logged_) {
            leader_first_cycle_logged_ = true;
            std::cout << "[leader " << arm_type_ << "] first cycle: gripper vel="
                      << (gripper_dof ? joint_gripper_velocities[0] : 0.0)
                      << " friction=" << (gripper_dof ? friction[arm_dof] : 0.0)
                      << " tau=" << (gripper_dof ? joint_gripper_state_torque[0].effort : 0.0)
                      << " | arm tau=";
            for (size_t i = 0; i < arm_dof; ++i)
                std::cout << (i ? "," : "") << joint_arm_state_torque[i].effort;
            std::cout << std::endl;
        }

        std::vector<MotorState> motor_arm_states =
            openarmjointconverter_->joint_to_motor(joint_arm_state_torque);
        std::vector<MotorState> motor_gripper_states =
            openarmgripperjointconverter_->joint_to_motor(joint_gripper_state_torque);

        // arm command mit param
        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_dof);
        for (size_t i = 0; i < arm_dof; ++i) {
            arm_cmds.emplace_back(
                openarm::damiao_motor::MITParam{0.0, 0.0, 0.0, 0.0, motor_arm_states[i].effort});
        }

        // gripper command mit param
        std::vector<openarm::damiao_motor::MITParam> gripper_cmds;
        gripper_cmds.reserve(gripper_dof);
        for (size_t i = 0; i < gripper_dof; ++i) {
            gripper_cmds.emplace_back(openarm::damiao_motor::MITParam{
                0.0, 0.0, 0.0, 0.0, motor_gripper_states[i].effort});
        }

        // send command to arm
        openarm_->get_arm().mit_control_all(arm_cmds);
        // send command to gripper
        openarm_->get_gripper().mit_control_all(gripper_cmds);

        openarm_->recv_all(200);

        return true;
    }
}

void Control::ComputeFriction(const double* velocity, double* friction, size_t index) {
    if (TANHFRIC) {
        const double amp_tmp = 1.0;
        const double coef_tmp = 0.1;

        const double v = velocity[index];
        const double Fc = Fc_.at(index);
        const double k = k_.at(index);
        const double Fv = Fv_.at(index);
        const double Fo = Fo_.at(index);

        friction[index] = amp_tmp * Fc * std::tanh(coef_tmp * k * v) + Fv * v + Fo;
    } else {
        friction[index] = velocity[index] * Dn_.at(index);
    }
}

bool Control::MoveToPose(const std::vector<double>& target_arm,
                         const std::vector<double>& target_gripper) {
    int nstep = 220;
    double alpha;

    std::vector<MotorState> arm_motor_states;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<MotorState> gripper_motor_states;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<JointState> joint_arm_now =
        openarmjointconverter_->motor_to_joint(arm_motor_states);
    std::vector<JointState> joint_hand_now =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

    std::vector<JointState> joint_arm_goal(NMOTORS - 1);
    for (size_t i = 0; i < NMOTORS - 1; ++i) {
        joint_arm_goal[i].position = (i < target_arm.size()) ? target_arm[i] : INITIAL_POSITION[i];
        joint_arm_goal[i].velocity = 0.0;
        joint_arm_goal[i].effort = 0.0;
    }

    std::vector<JointState> joint_hand_goal(joint_hand_now.size());
    for (size_t i = 0; i < joint_hand_goal.size(); ++i) {
        joint_hand_goal[i].position = (i < target_gripper.size()) ? target_gripper[i] : 0.0;
        joint_hand_goal[i].velocity = 0.0;
        joint_hand_goal[i].effort = 0.0;
    }

    std::vector<double> kp_arm_temp = {50, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
    std::vector<double> kd_arm_temp = {1.2, 1.2, 1.2, 1.2, 0.3, 0.2, 0.3};

    std::vector<double> kp_hand_temp = {10.0};
    std::vector<double> kd_hand_temp = {0.5};

    for (int step = 0; step < nstep; ++step) {
        alpha = static_cast<double>(step + 1) / nstep;

        std::vector<JointState> joint_arm_interp(NMOTORS - 1);
        for (size_t i = 0; i < NMOTORS - 1; ++i) {
            joint_arm_interp[i].position =
                joint_arm_goal[i].position * alpha + joint_arm_now[i].position * (1.0 - alpha);
            joint_arm_interp[i].velocity = 0.0;
        }

        std::vector<JointState> joint_hand_interp(joint_hand_goal.size());
        for (size_t i = 0; i < joint_hand_interp.size(); ++i) {
            joint_hand_interp[i].position =
                joint_hand_goal[i].position * alpha + joint_hand_now[i].position * (1.0 - alpha);
            joint_hand_interp[i].velocity = 0.0;
        }

        std::vector<MotorState> arm_motor_refs =
            openarmjointconverter_->joint_to_motor(joint_arm_interp);
        std::vector<MotorState> hand_motor_refs =
            openarmgripperjointconverter_->joint_to_motor(joint_hand_interp);

        std::vector<openarm::damiao_motor::MITParam> arm_cmds;
        arm_cmds.reserve(arm_motor_refs.size());
        for (size_t i = 0; i < arm_motor_refs.size(); ++i) {
            arm_cmds.emplace_back(openarm::damiao_motor::MITParam{kp_arm_temp[i], kd_arm_temp[i],
                                                                  arm_motor_refs[i].position,
                                                                  arm_motor_refs[i].velocity, 0.0});
        }

        std::vector<openarm::damiao_motor::MITParam> hand_cmds;
        hand_cmds.reserve(hand_motor_refs.size());
        for (size_t i = 0; i < hand_motor_refs.size(); ++i) {
            hand_cmds.emplace_back(openarm::damiao_motor::MITParam{
                kp_hand_temp[i], kd_hand_temp[i], hand_motor_refs[i].position,
                hand_motor_refs[i].velocity, 0.0});
        }

        openarm_->get_arm().mit_control_all(arm_cmds);
        openarm_->get_gripper().mit_control_all(hand_cmds);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        openarm_->recv_all();

        std::vector<MotorState> arm_motor_states_now;
        for (const auto& motor : openarm_->get_arm().get_motors()) {
            arm_motor_states_now.push_back({motor.get_position(), motor.get_velocity(), 0.0});
        }
        std::vector<MotorState> gripper_motor_states_now;
        for (const auto& motor : openarm_->get_gripper().get_motors()) {
            gripper_motor_states_now.push_back({motor.get_position(), motor.get_velocity(), 0.0});
        }
        std::vector<JointState> joint_arm_now_step =
            openarmjointconverter_->motor_to_joint(arm_motor_states_now);
        std::vector<JointState> joint_hand_now_step =
            openarmgripperjointconverter_->motor_to_joint(gripper_motor_states_now);

        robot_state_->arm_state().set_all_responses(joint_arm_now_step);
        robot_state_->hand_state().set_all_responses(joint_hand_now_step);
    }

    std::vector<MotorState> arm_motor_states_final;
    for (const auto& motor : openarm_->get_arm().get_motors()) {
        arm_motor_states_final.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<MotorState> gripper_motor_states_final;
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
        gripper_motor_states_final.push_back({motor.get_position(), motor.get_velocity(), 0.0});
    }

    std::vector<JointState> joint_arm_final =
        openarmjointconverter_->motor_to_joint(arm_motor_states_final);
    std::vector<JointState> joint_hand_final =
        openarmgripperjointconverter_->motor_to_joint(gripper_motor_states_final);

    robot_state_->arm_state().set_all_references(joint_arm_final);
    robot_state_->hand_state().set_all_references(joint_hand_final);
    robot_state_->arm_state().set_all_responses(joint_arm_final);
    robot_state_->hand_state().set_all_responses(joint_hand_final);

    return true;
}

// Move to the fixed home pose (INITIAL_POSITION, grippers open).
bool Control::AdjustPosition(void) {
    std::vector<double> target_arm(INITIAL_POSITION, INITIAL_POSITION + (NMOTORS - 1));
    std::vector<double> target_gripper(1, 0.0);
    return MoveToPose(target_arm, target_gripper);
}

bool Control::DetectVibration(const double* velocity, bool* what_axis) {
    bool vibration_detected = false;

    for (int i = 0; i < NJOINTS; ++i) {
        what_axis[i] = false;

        velocity_buffer_[i].push_back(velocity[i]);
        if (velocity_buffer_[i].size() > VEL_WINDOW_SIZE) velocity_buffer_[i].pop_front();

        if (velocity_buffer_[i].size() < VEL_WINDOW_SIZE) continue;

        double mean = std::accumulate(velocity_buffer_[i].begin(), velocity_buffer_[i].end(), 0.0) /
                      velocity_buffer_[i].size();

        double var = 0.0;
        for (double v : velocity_buffer_[i]) {
            var += (v - mean) * (v - mean);
        }

        double stddev = std::sqrt(var / velocity_buffer_[i].size());

        if (stddev > VIB_THRESHOLD) {
            what_axis[i] = true;
            vibration_detected = true;
            std::cout << "[VIBRATION] Joint " << i << " stddev: " << stddev << std::endl;
        }
    }

    return vibration_detected;
}