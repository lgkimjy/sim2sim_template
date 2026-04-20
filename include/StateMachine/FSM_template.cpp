#include "StateMachine/FSM_template.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iostream>
#include <string>

TemplateRlState::TemplateRlState(RobotData& robot, const RobotDefinition& def)
    : robot_(robot),
      def_(def)
{
}

void TemplateRlState::onEnter()
{
    readConfig(CMAKE_SOURCE_DIR "/config/template_config.yaml");

    policy_.setIoNames(robot_.policy.input_name, robot_.policy.output_name);
    policy_.load(robot_.policy.onnx_path);
    robot_.ctrl.jvel_d.setZero();
    robot_.ctrl.tau_cmd.setZero();
}

void TemplateRlState::runNominal()
{
    Eigen::VectorXd raw = Eigen::VectorXd::Zero(def_.dof);
    if (policy_.isLoaded()) raw = policy_.forward(makeObservation()).head(def_.dof);
    applyPolicyAction(raw.unaryExpr([](double x) { return std::clamp(x, -1.0, 1.0); }));

    robot_.ctrl.tau_cmd = robot_.param.Kp.cwiseProduct(robot_.ctrl.jpos_d - robot_.fbk.q)
                        - robot_.param.Kd.cwiseProduct(robot_.fbk.dq);
}

Eigen::VectorXd TemplateRlState::makeObservation() const
{
    const Eigen::Vector3d gravity_b = robot_.fbk.base_quat.inverse() * Eigen::Vector3d(0.0, 0.0, -1.0);
    Eigen::Vector3d cmd;
    cmd << robot_.ctrl.lin_vel_d.x(), robot_.ctrl.lin_vel_d.y(), robot_.ctrl.ang_vel_d.z();
    // gravity(3) + omega(3) + q err(dof) + dq(dof) + cmd(3)
    Eigen::VectorXd obs(3 + 3 + def_.dof + def_.dof + 3);
    obs << gravity_b, robot_.fbk.base_omega, robot_.fbk.q - robot_.param.default_jpos, robot_.fbk.dq, cmd;
    return obs;
}

void TemplateRlState::applyPolicyAction(const Eigen::VectorXd& action)
{
    const double s = robot_.policy.action_scale;
    robot_.ctrl.jpos_d = robot_.param.default_jpos;
    for (int i = 0; i < def_.dof; ++i) {
        const int j = def_.policy_to_robot[i];
        robot_.ctrl.jpos_d(j) = robot_.param.default_jpos(j) + s * action(i);
    }
}

void TemplateRlState::checkTransition() {}
void TemplateRlState::runTransition() {}

void TemplateRlState::readConfig(const std::string& config_file)
{
    std::cout << "[ TemplateRlState ] readConfig: " << config_file << std::endl;
    YAML::Node config = YAML::LoadFile(config_file);

    robot_.param.Kp.resize(def_.dof);
    robot_.param.Kd.resize(def_.dof);
    robot_.param.default_jpos.resize(def_.dof);

    for (int i = 0; i < def_.dof; ++i) {
        robot_.param.Kp(i) = config["Robot"]["Kp"][i].as<double>();
        robot_.param.Kd(i) = config["Robot"]["Kd"][i].as<double>();
        robot_.param.default_jpos(i) = config["default_jpos"][i].as<double>();
    }

    robot_.policy.onnx_path = config["policy"]["path"].as<std::string>();
    robot_.policy.input_name = config["policy"]["input_name"].as<std::string>();
    robot_.policy.output_name = config["policy"]["output_name"].as<std::string>();
    robot_.policy.action_scale = config["policy"]["action_scale"].as<double>();

    robot_.ctrl.lin_vel_d.x() = config["command"]["vx"].as<double>();
    robot_.ctrl.lin_vel_d.y() = config["command"]["vy"].as<double>();
    robot_.ctrl.ang_vel_d.z() = config["command"]["wz"].as<double>();
}