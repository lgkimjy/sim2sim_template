#!/usr/bin/env python3

import argparse
import re
from pathlib import Path


def normalize_task_name(task: str) -> str:
    tokens = re.findall(r"[A-Za-z0-9]+", task)
    if not tokens:
        raise ValueError("Task name must contain at least one alphanumeric character.")
    return "".join(token[:1].upper() + token[1:] for token in tokens)


def write_text(path: Path, content: str, force: bool) -> None:
    if path.exists() and not force:
        raise FileExistsError(f"{path} already exists. Re-run with --force to overwrite it.")
    path.write_text(content, encoding="utf-8")


def insert_before(path: Path, marker: str, block: str) -> None:
    text = path.read_text(encoding="utf-8")
    if block in text:
        return
    if marker not in text:
        raise RuntimeError(f"Failed to find marker in {path}: {marker}")
    updated = text.replace(marker, f"{block}{marker}", 1)
    path.write_text(updated, encoding="utf-8")


def insert_after_last_regex(path: Path, pattern: str, block: str) -> None:
    text = path.read_text(encoding="utf-8")
    if block in text:
        return
    matches = list(re.finditer(pattern, text, flags=re.MULTILINE))
    if not matches:
        raise RuntimeError(f"Failed to find insertion point in {path}: {pattern}")
    pos = matches[-1].end()
    updated = text[:pos] + block + text[pos:]
    path.write_text(updated, encoding="utf-8")


def replace_regex(path: Path, pattern: str, repl: str) -> None:
    text = path.read_text(encoding="utf-8")
    updated, count = re.subn(pattern, repl, text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise RuntimeError(f"Failed to update {path} with pattern: {pattern}")
    path.write_text(updated, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a new sim2sim FSM task scaffold.")
    parser.add_argument("--task", required=True, help="Task name, for example G1-Vel-Track")
    parser.add_argument("--force", action="store_true", help="Overwrite generated files if they already exist")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    task_name = normalize_task_name(args.task)
    class_name = f"{task_name}State"
    enum_name = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", task_name).upper()

    header_path = repo_root / "include" / "StateMachine" / f"FSM_{task_name}.hpp"
    source_path = repo_root / "include" / "StateMachine" / f"FSM_{task_name}.cpp"
    config_path = repo_root / "config" / f"{task_name}_config.yaml"
    ctrl_hpp_path = repo_root / "include" / "StateMachine" / "StateMachineCtrl.hpp"
    ctrl_cpp_path = repo_root / "include" / "StateMachine" / "StateMachineCtrl.cpp"

    header_text = f"""#pragma once

#include "RobotDefinition.hpp"
#include "RobotStates.hpp"
#include "StateMachine/States.hpp"

#include <string>

class {class_name} final : public States {{
public:
    {class_name}(RobotData& robot, const RobotDefinition& def);

    void onEnter() override;
    void runNominal() override;
    void checkTransition() override;
    void runTransition() override;

private:
    void readConfig(const std::string& config_file);

    RobotData& robot_;
    const RobotDefinition& def_;
}};
"""

    source_text = f"""#include "StateMachine/FSM_{task_name}.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>

{class_name}::{class_name}(RobotData& robot, const RobotDefinition& def)
    : robot_(robot),
      def_(def)
{{
}}

void {class_name}::onEnter()
{{
    readConfig(CMAKE_SOURCE_DIR "/config/{task_name}_config.yaml");
    robot_.ctrl.jpos_d = robot_.fbk.q;
    robot_.ctrl.jvel_d.setZero();
    robot_.ctrl.tau_cmd.setZero();
}}

void {class_name}::runNominal()
{{
    robot_.ctrl.tau_cmd.setZero();
}}

void {class_name}::checkTransition() {{}}
void {class_name}::runTransition() {{}}

void {class_name}::readConfig(const std::string& config_file)
{{
    std::cout << "[ {class_name} ] readConfig: " << config_file << std::endl;
    const YAML::Node config = YAML::LoadFile(config_file);
    (void)config;
    (void)def_;
}}
"""

    config_text = f"""policy:
  name: "{args.task}"
  path: ""

command:
  vx: 0.0
  vy: 0.0
  wz: 0.0

notes:
  description: "Fill in task-specific gains, commands, and policy settings here."
"""

    write_text(header_path, header_text, args.force)
    write_text(source_path, source_text, args.force)
    write_text(config_path, config_text, args.force)

    insert_before(
        ctrl_hpp_path,
        "    NUM_STATE\n",
        f"    {enum_name},\n",
    )
    insert_after_last_regex(
        ctrl_cpp_path,
        r'^#include "StateMachine/FSM_[^"]+\.hpp"\n',
        f'#include "StateMachine/FSM_{task_name}.hpp"\n',
    )
    insert_after_last_regex(
        ctrl_cpp_path,
        r"^\s*state_list_\[StateList::[A-Z0-9_]+\] = new [A-Za-z0-9_]+\(robot, def\);\n",
        f"    state_list_[StateList::{enum_name}] = new {class_name}(robot, def);\n",
    )
    replace_regex(
        ctrl_cpp_path,
        r"current_state_ = state_list_\[StateList::[A-Z0-9_]+\];",
        f"current_state_ = state_list_[StateList::{enum_name}];",
    )

    created = [header_path, source_path, config_path, ctrl_hpp_path, ctrl_cpp_path]
    for path in created:
        print(path.relative_to(repo_root))


if __name__ == "__main__":
    main()
