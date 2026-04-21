#include "StateMachine/StateMachineCtrl.hpp"

#include "StateMachine/FSM_Template.hpp"

#include <iostream>

StateMachineCtrl::StateMachineCtrl(RobotData& robot, const RobotDefinition& def)
{
    state_list_.resize(StateList::NUM_STATE, nullptr);
    state_list_[StateList::RL_TEMPLATE] = new TemplateRlState(robot, def);

    current_state_ = nullptr;
    next_state_ = nullptr;
    std::cout << "[ StateMachineCtrl ] constructed" << std::endl;
}

StateMachineCtrl::~StateMachineCtrl()
{
    for (States* s : state_list_) {
        delete s;
    }
}

void StateMachineCtrl::initialize()
{
    current_state_ = state_list_[StateList::RL_TEMPLATE];
    next_state_ = current_state_;
    first_run_ = true;
}

void StateMachineCtrl::runState()
{
    if (first_run_) {
        current_state_->onEnter();
        first_run_ = false;
    }
    current_state_->runNominal();
}
