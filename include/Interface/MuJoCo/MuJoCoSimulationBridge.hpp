#pragma once

#include "Interface/MuJoCo/SimulationInterface.hpp"
#include "RobotDefinition.hpp"
#include "RobotStates.hpp"
#include "StateMachine/StateMachineCtrl.hpp"

class MuJoCoSimulationBridge final : public SimulationInterface {
public:
    MuJoCoSimulationBridge(RobotDefinition robot, const std::string& scene_file);

private:
    void Initialize() override;
    void UpdateSystemObserver() override;
    void UpdateUserInput() override;
    void UpdateControlCommand() override;
    void UpdateSystemVisualInfo() override {}

    RobotDefinition robot_def_;
    RobotData robot_;
    StateMachineCtrl state_machine_;
};
