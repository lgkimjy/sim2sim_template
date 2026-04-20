#pragma once

#include <Eigen/Dense>
#include <string>

// --- Feedback (sensors / sim truth): log under e.g. "/fbk/..." for HDF5 ---
struct RobotFeedback {
    Eigen::Quaterniond base_quat = Eigen::Quaterniond::Identity();
    Eigen::Vector3d base_omega = Eigen::Vector3d::Zero();
    Eigen::VectorXd q;
    Eigen::VectorXd dq;

    explicit RobotFeedback(int dof = 0) : q(Eigen::VectorXd::Zero(dof)), dq(Eigen::VectorXd::Zero(dof)) {}
};

// --- Commands & actuator outputs: motion targets + torque (not "motor struct" only) ---
struct RobotCtrl {
    Eigen::Vector3d lin_vel_d = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_d = Eigen::Vector3d::Zero();

    Eigen::VectorXd jpos_d;
    Eigen::VectorXd jvel_d;
    Eigen::VectorXd tau_cmd;

    explicit RobotCtrl(int dof = 0)
        : jpos_d(Eigen::VectorXd::Zero(dof)),
          jvel_d(Eigen::VectorXd::Zero(dof)),
          tau_cmd(Eigen::VectorXd::Zero(dof)) {}
};

// --- Parameters from config (PD, nominal posture); joint order = RobotDefinition::kJointNames / joint_names ---
struct RobotParam {
    Eigen::VectorXd Kp;
    Eigen::VectorXd Kd;
    Eigen::VectorXd default_jpos;

    explicit RobotParam(int dof = 0)
        : Kp(Eigen::VectorXd::Zero(dof)),
          Kd(Eigen::VectorXd::Zero(dof)),
          default_jpos(Eigen::VectorXd::Zero(dof)) {}
};

struct PolicyConfig {
    std::string onnx_path;
    std::string input_name = "obs";
    std::string output_name = "actions";
    double action_scale = 0.25;
};

struct RobotData {
    RobotFeedback fbk;
    RobotCtrl ctrl;
    RobotParam param;
    PolicyConfig policy;

    explicit RobotData(int dof = 0) : fbk(dof), ctrl(dof), param(dof) {}
};
