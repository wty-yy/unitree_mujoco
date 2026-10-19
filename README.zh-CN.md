<div align="center">
  <h1>Unitree G1 MuJoCo</h1>
  <p><strong>基于 <a href="https://github.com/unitreerobotics/unitree_mujoco">Unitree MuJoCo</a> 的 G1 模型、地形场景与 C++ 仿真器，通过 Unitree SDK2 通信</strong></p>
  <p><a href="README.md">🌎 English</a>&nbsp;&nbsp;·&nbsp;&nbsp;<strong>🇨🇳 中文</strong></p>
</div>

## 模型与场景

使用 G1 29DOF Mode 15 模型，默认加载不带鞋的完整碰撞模型和平地场景 `flat.xml`。

- `g1_29dof_mode_15_complete_collisions.xml`：不带鞋，`default` 或 `complete`
- `g1_29dof_mode_15_complete_collisions_shoes.xml`：带鞋，`complete-shoes`

34 个机身网格由两个模型共用，6 个鞋子网格位于 `unitree_robots/g1/meshes/shoes/`。

| 场景 | 模型 | 地形 |
| --- | --- | --- |
| `flat.xml` | G1 29DOF | 平地，默认场景 |
| `stairs.xml` | G1 29DOF | 楼梯 |
| `cross_stairs.xml` | G1 29DOF | 交叉楼梯 |
| `cross_slope.xml` | G1 29DOF | 交叉斜坡 |
| `race_track.xml` | G1 29DOF | 综合赛道 |
| `scene_29dof.xml` | G1 29DOF | 基础平地 |

场景位于 [`unitree_robots/g1/`](unitree_robots/g1/)。地形尺寸标注可通过 MuJoCo 的 geom group 1 显示开关查看。

## 安装与编译

安装 [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)，需要包含 `unitree/dds_wrapper/robots/g1/` 头文件。CMake 搜索 `/opt/unitree_robotics` 和标准安装前缀。

从 [MuJoCo 发行页](https://github.com/google-deepmind/mujoco/releases) 下载包含 `include/`、`lib/` 和 `simulate/` 源码的 Linux 发行包并解压。

```bash
git clone --single-branch --branch g1 https://github.com/wty-yy/unitree_mujoco.git unitree_mujoco_g1
cd unitree_mujoco_g1
sudo apt install cmake g++ libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev libfmt-dev

# 将路径替换为本地 MuJoCo 安装目录
ln -s /path/to/mujoco-[version] simulate/mujoco
cmake -S simulate -B simulate/build -DCMAKE_BUILD_TYPE=Release
cmake --build simulate/build -j$(nproc)
```

## 运行

默认启用手柄，启动前连接 Xbox 或 Switch 手柄；未连接手柄时，将 [`simulate/config.yaml`](simulate/config.yaml) 中的 `use_joystick` 设为 `0`。仿真窗口需要图形显示环境。

```bash
# 查看手柄输入
./simulate/build/jstest

# 默认 G1 29DOF Mode 15 平地场景
./simulate/build/unitree_mujoco
./simulate/build/unitree_mujoco -s stairs.xml
./simulate/build/unitree_mujoco -s stairs.xml -m complete-shoes -i 0 -n lo
./simulate/build/unitree_mujoco --help
```

| 参数 | 配置项 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `-s`、`--scene` | `robot_scene` | `flat.xml` | 相对路径从 `unitree_robots/g1/` 解析，也支持绝对路径 |
| `-m`、`--model` | `robot_model` | `default` | 模型：`default`、`complete`、`complete-shoes` 或模型 XML 路径 |
| `-i`、`--domain_id` | `domain_id` | `0` | DDS 域 ID |
| `-n`、`--network` | `interface` | `lo` | DDS 网络接口 |
| `-h`、`--help` | — | — | 显示帮助并退出 |

命令行参数覆盖 YAML 配置。配置和场景路径根据可执行文件位置解析，可从其他工作目录启动；构建目录使用 `simulate/build/`。

仿真器接收外部控制程序的底层指令。控制程序应使用相同的 DDS 域和网络接口，发送 `unitree_hg::msg::dds_::LowCmd_`，并使用关节角控制模式 `mode_pr = 0`。

## 配置与挂带

以下选项在 [`simulate/config.yaml`](simulate/config.yaml) 中设置：

| 配置项 | 默认值 | 说明 |
| --- | --- | --- |
| `use_joystick` | `1` | 启用手柄输入，`0` 关闭 |
| `joystick_type` | `xbox` | 手柄布局：`xbox` 或 `switch` |
| `joystick_device` | `/dev/input/js0` | 手柄设备路径 |
| `joystick_bits` | `16` | 摇杆精度位数 |
| `print_scene_information` | `1` | 输出连杆、关节、执行器和传感器信息 |
| `enable_elastic_band` | `0` | G1 躯干虚拟挂带，`1` 启用，`0` 关闭 |

虚拟挂带默认关闭。将 `enable_elastic_band` 设为 `1` 后，挂带初始连接，便于控制器启动与调试。仿真窗口内按 `9` 连接或松开，按 `7` 或方向键 `↑` 缩短挂带、抬高机器人，按 `8` 或 `↓` 加长挂带、降低机器人。

## DDS 接口

| 话题 | 消息类型 | 方向与内容 |
| --- | --- | --- |
| `rt/lowcmd` | `unitree_hg::msg::dds_::LowCmd_` | 订阅：关节位置、速度、增益和前馈力矩 |
| `rt/lowstate` | `unitree_hg::msg::dds_::LowState_` | 发布：电机状态、主 IMU、手柄数据和仿真 tick |
| `rt/secondary_imu` | `unitree_hg::msg::dds_::IMUState_` | 发布：胸部 IMU |
| `rt/lf/bmsstate` | `unitree_hg::msg::dds_::BmsState_` | 发布：模拟电池电量，固定 `soc = 100` |
| `rt/sportmodestate` | `unitree_go::msg::dds_::SportModeState_` | 发布：主 IMU 位置和线速度，供仿真分析 |
| `rt/wirelesscontroller` | `unitree_go::msg::dds_::WirelessController_` | 启用手柄时发布：摇杆和按键 |

电机通信使用 G1 的 `unitree_hg` 类型；位置遥测和手柄沿用 SDK 的共享 `unitree_go` 消息格式。桥接循环周期为 `1 ms`。

DDS 状态的 `mode_machine = 5`，电机按 29 个关节的顺序收发，详见[仿真关节索引](unitree_robots/g1/g1_joint_index_dds.md)。

## 目录结构

```text
unitree_mujoco_g1/
├── simulate/
│   ├── CMakeLists.txt           # C++ 构建配置
│   ├── config.yaml              # 场景、DDS、手柄与挂带配置
│   ├── mujoco/                  # 本地 MuJoCo 发行包的软链接
│   └── src/                     # 仿真、G1 DDS 桥接与手柄支持
└── unitree_robots/g1/            # Mode 15 MJCF 模型与地形场景
    ├── meshes/                  # 共用机身网格
    │   └── shoes/               # 鞋子网格
    └── urdf/                    # 源 URDF 与 SRDF
```

## 许可证

原创代码采用 [MIT License](LICENSE)。源自 Unitree MuJoCo 的代码保留 BSD 3-Clause License，详见同一文件。第三方代码遵循各自的许可证。
