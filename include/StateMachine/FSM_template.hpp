#pragma once

#include "Controller/OnnxParser/OnnxPolicy.hpp"
#include "RobotDefinition.hpp"
#include "RobotStates.hpp"
#include "StateMachine/States.hpp"

#include <string>

// RL task: yaml read in readConfig() (G1 FSM style), first onEnter.
class TemplateRlState final : public States {
public:
    TemplateRlState(RobotData& robot, const RobotDefinition& def);

    void onEnter() override;
    void runNominal() override;
    void checkTransition() override;
    void runTransition() override;

private:
    void readConfig(const std::string& config_file);

    RobotData& robot_;
    const RobotDefinition& def_;
    OnnxPolicy policy_;

    Eigen::VectorXd makeObservation() const;
    void applyPolicyAction(const Eigen::VectorXd& action);
};
