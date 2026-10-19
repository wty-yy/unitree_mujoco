<div align="center">
  <h1>Unitree G1 MuJoCo</h1>
  <p><strong>G1 models, terrain scenes, and a C++ simulator based on <a href="https://github.com/unitreerobotics/unitree_mujoco">Unitree MuJoCo</a>, communicating through Unitree SDK2</strong></p>
  <p><strong>🌎 English</strong>&nbsp;&nbsp;·&nbsp;&nbsp;<a href="README.zh-CN.md">🇨🇳 中文</a></p>
</div>

## Models and scenes

Uses the G1 29DOF Mode 15 model. The default is the full-collision model without shoes in `flat.xml`.

- `g1_29dof_mode_15_complete_collisions.xml`: without shoes, selected by `default` or `complete`
- `g1_29dof_mode_15_complete_collisions_shoes.xml`: with shoes, selected by `complete-shoes`

Both models share 34 body meshes. Six shoe meshes are under `unitree_robots/g1/meshes/shoes/`.

| Scene | Model | Terrain |
| --- | --- | --- |
| `flat.xml` | G1 29DOF | Flat ground, default scene |
| `stairs.xml` | G1 29DOF | Stairs |
| `cross_stairs.xml` | G1 29DOF | Crossing stairs |
| `cross_slope.xml` | G1 29DOF | Crossing slopes |
| `race_track.xml` | G1 29DOF | Mixed terrain course |
| `scene_29dof.xml` | G1 29DOF | Basic flat ground |

Scenes are under [`unitree_robots/g1/`](unitree_robots/g1/). Toggle geom group 1 in MuJoCo to display terrain dimension labels.

## Installation and build

Install [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2) with the `unitree/dds_wrapper/robots/g1/` headers. CMake searches `/opt/unitree_robotics` and standard installation prefixes.

Download and extract a Linux package from the [MuJoCo releases page](https://github.com/google-deepmind/mujoco/releases) containing `include/`, `lib/`, and the `simulate/` source code.

```bash
git clone --single-branch --branch g1 https://github.com/wty-yy/unitree_mujoco.git unitree_mujoco_g1
cd unitree_mujoco_g1
sudo apt install cmake g++ libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev libfmt-dev

# Replace with your local MuJoCo installation directory
ln -s /path/to/mujoco-[version] simulate/mujoco
cmake -S simulate -B simulate/build -DCMAKE_BUILD_TYPE=Release
cmake --build simulate/build -j$(nproc)
```

## Running

Gamepad input is enabled by default. Connect an Xbox or Switch gamepad before starting, or set `use_joystick: 0` in [`simulate/config.yaml`](simulate/config.yaml). The simulator window requires a graphical display.

```bash
# Inspect gamepad input
./simulate/build/jstest

# Default G1 29DOF Mode 15 flat scene
./simulate/build/unitree_mujoco
./simulate/build/unitree_mujoco -s stairs.xml
./simulate/build/unitree_mujoco -s stairs.xml -m complete-shoes -i 0 -n lo
./simulate/build/unitree_mujoco --help
```

| Option | Configuration key | Default | Description |
| --- | --- | --- | --- |
| `-s`, `--scene` | `robot_scene` | `flat.xml` | Relative to `unitree_robots/g1/`; absolute paths are also supported |
| `-m`, `--model` | `robot_model` | `default` | Model: `default`, `complete`, `complete-shoes`, or a model XML path |
| `-i`, `--domain_id` | `domain_id` | `0` | DDS domain ID |
| `-n`, `--network` | `interface` | `lo` | DDS network interface |
| `-h`, `--help` | — | — | Print help and exit |

Command-line options override YAML settings. Configuration and scene paths resolve from the executable location, so the simulator can start from another working directory. Use `simulate/build/` as the build directory.

The simulator receives low-level commands from an external controller. Use the same DDS domain and network interface, publish `unitree_hg::msg::dds_::LowCmd_`, and select joint-angle control with `mode_pr = 0`.

## Configuration and suspension

Set these options in [`simulate/config.yaml`](simulate/config.yaml):

| Key | Default | Description |
| --- | --- | --- |
| `use_joystick` | `1` | Enable gamepad input; `0` disables it |
| `joystick_type` | `xbox` | Gamepad layout: `xbox` or `switch` |
| `joystick_device` | `/dev/input/js0` | Gamepad device path |
| `joystick_bits` | `16` | Stick resolution in bits |
| `print_scene_information` | `1` | Print bodies, joints, actuators, and sensors |
| `enable_elastic_band` | `0` | G1 torso suspension band; `1` enables it, `0` disables it |

The virtual band is disabled by default. Set `enable_elastic_band: 1` to start with the band attached for controller initialization and debugging. In the simulator window, press `9` to attach or release it, `7` or `↑` to shorten the band and raise the robot, and `8` or `↓` to lengthen it and lower the robot.

## DDS interfaces

| Topic | Message type | Direction and content |
| --- | --- | --- |
| `rt/lowcmd` | `unitree_hg::msg::dds_::LowCmd_` | Subscribe: joint positions, velocities, gains, and feedforward torques |
| `rt/lowstate` | `unitree_hg::msg::dds_::LowState_` | Publish: motor states, primary IMU, gamepad data, and simulation tick |
| `rt/secondary_imu` | `unitree_hg::msg::dds_::IMUState_` | Publish: chest IMU |
| `rt/lf/bmsstate` | `unitree_hg::msg::dds_::BmsState_` | Publish: simulated battery charge, fixed at `soc = 100` |
| `rt/sportmodestate` | `unitree_go::msg::dds_::SportModeState_` | Publish: primary IMU position and linear velocity for simulation analysis |
| `rt/wirelesscontroller` | `unitree_go::msg::dds_::WirelessController_` | Publish when gamepad input is enabled: sticks and buttons |

Motor communication uses G1 `unitree_hg` types. Position telemetry and gamepad input retain the SDK's shared `unitree_go` message format. The bridge loop runs every `1 ms`.

DDS states report `mode_machine = 5`. Motor commands and states follow the 29-joint ordering in the [simulation joint indices](unitree_robots/g1/g1_joint_index_dds.md).

## Repository layout

```text
unitree_mujoco_g1/
├── simulate/
│   ├── CMakeLists.txt           # C++ build configuration
│   ├── config.yaml              # Scene, DDS, gamepad, and suspension settings
│   ├── mujoco/                  # Symlink to a local MuJoCo release
│   └── src/                     # Simulation, G1 DDS bridge, and gamepad support
└── unitree_robots/g1/            # Mode 15 MJCF models and terrain scenes
    ├── meshes/                  # Shared body meshes
    │   └── shoes/               # Shoe meshes
    └── urdf/                    # Source URDF and SRDF files
```

## License

Original code is licensed under the [MIT License](LICENSE). Code derived from Unitree MuJoCo retains its BSD 3-Clause License in the same file. Third-party code follows its respective licenses.
