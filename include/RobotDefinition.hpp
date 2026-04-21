#pragma once

#include <array>
#include <string>
#include <vector>

// Header-only like G1Definition.hpp. Vectors mirror constexpr tables for APIs that want std::vector; filled in ctor.
struct RobotDefinition {
    static constexpr int kNumActuatedJoints = 6;
    static constexpr int kQposOffset = 7;
    static constexpr int kQvelOffset = 6;
    static constexpr int kCtrlOffset = 0;

    static constexpr std::array<int, kNumActuatedJoints> kPolicyToActuatorIndex = {0, 1, 2, 3, 4, 5};

    static constexpr std::array<const char*, kNumActuatedJoints> kJointNames = {"left_hip_roll",
        "left_hip_pitch",
        "left_knee",
        "right_hip_roll",
        "right_hip_pitch",
        "right_knee"};

    int dof = kNumActuatedJoints;
    int qpos_offset = kQposOffset;
    int qvel_offset = kQvelOffset;
    int ctrl_offset = kCtrlOffset;
    std::string model_xml = "model/robot/scene.xml";
    std::vector<std::string> joint_names;
    std::vector<int> policy_to_robot;

    RobotDefinition()
    {
        joint_names.reserve(kNumActuatedJoints);
        for (int i = 0; i < kNumActuatedJoints; ++i) {
            joint_names.emplace_back(kJointNames[static_cast<size_t>(i)]);
        }
        policy_to_robot.assign(kPolicyToActuatorIndex.begin(), kPolicyToActuatorIndex.end());
    }
};
