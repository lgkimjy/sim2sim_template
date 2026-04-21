# sim2sim_template

ONNX policy deploy on MuJoCo.

## Build

```bash
git clone <repo>
cd <repo>
git clone https://github.com/google-deepmind/mujoco.git include/3rd-parties/mujoco
mkdir build
cd build
cmake ..
make -j
```

MuJoCo is not fetched automatically. Clone [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco) into `include/3rd-parties/mujoco`.

The deploy binary is:

```bash
./sim2sim_deploy
```

## Create A Task

```bash
python3 scripts/create_task.py --task=G1-Vel-Track
```

This creates:

- `include/StateMachine/FSM_G1VelTrack.hpp`
- `include/StateMachine/FSM_G1VelTrack.cpp`
- `config/G1VelTrack_config.yaml`
- `include/StateMachine/StateMachineCtrl.hpp` active enum update
- `include/StateMachine/StateMachineCtrl.cpp` include and active state update

After that, rerun `cmake ..` and `make -j`. The new FSM source is picked up automatically, and the generated task becomes the active state in `StateMachineCtrl`.
