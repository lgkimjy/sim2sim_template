#pragma once

#include "RobotDefinition.hpp"
#include "RobotStates.hpp"
#include "StateMachine/States.hpp"

#include <vector>

enum StateList {
    PASSIVE = 0,
    RL_TEMPLATE = 1,
    NUM_STATE
};

class StateMachineCtrl {
public:
    StateMachineCtrl(RobotData& robot, const RobotDefinition& def);
    ~StateMachineCtrl();

    StateMachineCtrl(const StateMachineCtrl&) = delete;
    StateMachineCtrl& operator=(const StateMachineCtrl&) = delete;

    void initialize();
    void runState();

    std::vector<States*> state_list_;
    States* current_state_ = nullptr;
    States* next_state_ = nullptr;

private:
    bool first_run_ = true;
};
