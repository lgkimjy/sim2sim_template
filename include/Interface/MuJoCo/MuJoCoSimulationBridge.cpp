#include "Interface/MuJoCo/MuJoCoSimulationBridge.hpp"

#include <algorithm>
#include <iostream>

MuJoCoSimulationBridge::MuJoCoSimulationBridge(RobotDefinition robot, const std::string& scene_file)
    : SimulationInterface(scene_file),
      robot_def_(std::move(robot)),
      robot_(robot_def_.dof),
      state_machine_(robot_, robot_def_)
{
}

void MuJoCoSimulationBridge::Initialize()
{
    state_machine_.initialize();
}

void MuJoCoSimulationBridge::UpdateSystemObserver()
{
    if (!mjData_) return;

    robot_.fbk.base_quat.w() = mjData_->qpos[3];
    robot_.fbk.base_quat.x() = mjData_->qpos[4];
    robot_.fbk.base_quat.y() = mjData_->qpos[5];
    robot_.fbk.base_quat.z() = mjData_->qpos[6];
    robot_.fbk.base_omega << mjData_->qvel[3], mjData_->qvel[4], mjData_->qvel[5];

    for (int i = 0; i < robot_def_.dof; ++i) {
        robot_.fbk.q(i) = mjData_->qpos[i + robot_def_.qpos_offset];
        robot_.fbk.dq(i) = mjData_->qvel[i + robot_def_.qvel_offset];
    }
}

void MuJoCoSimulationBridge::UpdateUserInput()
{
    robot_.ctrl.lin_vel_d.x() = mjSim_->lin_vel_d.x();
    robot_.ctrl.lin_vel_d.y() = mjSim_->lin_vel_d.y();
    robot_.ctrl.lin_vel_d.z() = 0.0;
    robot_.ctrl.ang_vel_d.x() = 0.0;
    robot_.ctrl.ang_vel_d.y() = 0.0;
    robot_.ctrl.ang_vel_d.z() = mjSim_->ang_vel_d.z();
}

void MuJoCoSimulationBridge::UpdateControlCommand()
{
    state_machine_.runState();
    for (int i = 0; i < std::min(robot_def_.dof, mjModel_->nu); ++i) {
        mjData_->ctrl[i + robot_def_.ctrl_offset] = robot_.ctrl.tau_cmd(i);
    }
}
