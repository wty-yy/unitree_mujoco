<div align="center">
  <h1>Unitree Go2 MuJoCo</h1>
  <p><strong>基于 <a href="https://github.com/unitreerobotics/unitree_mujoco">unitreerobotics/unitree_mujoco</a> 的 Go2 模型、场景与 C++ MuJoCo 仿真器，通过 Unitree SDK2 通信</strong></p>
  <p><a href="README.md">🌎 English</a>&nbsp;&nbsp;·&nbsp;&nbsp;<strong>🇨🇳 中文</strong></p>
</div>

<table>
  <tr>
    <td align="center" width="33%">
      <a href="asset/figures/flat.png"><img src="asset/figures/flat.png" alt="flat" width="100%"></a><br>
      <code>flat</code>
    </td>
    <td align="center" width="33%">
      <a href="asset/figures/stairs.png"><img src="asset/figures/stairs.png" alt="stairs" width="100%"></a><br>
      <code>stairs</code>
    </td>
    <td align="center" width="33%">
      <a href="asset/figures/cross_stairs.png"><img src="asset/figures/cross_stairs.png" alt="cross_stairs" width="100%"></a><br>
      <code>cross_stairs</code>
    </td>
  </tr>
  <tr>
    <td align="center" width="33%">
      <a href="asset/figures/cross_slope.png"><img src="asset/figures/cross_slope.png" alt="cross_slope" width="100%"></a><br>
      <code>cross_slope</code>
    </td>
    <td align="center" width="33%">
      <a href="asset/figures/race_track.png"><img src="asset/figures/race_track.png" alt="race_track" width="100%"></a><br>
      <code>race_track</code>
    </td>
    <td align="center" width="33%">
      <a href="asset/figures/racing.png"><img src="asset/figures/racing.png" alt="racing" width="100%"></a><br>
      <code>racing</code>
    </td>
  </tr>
</table>

> 使用 F7 按三次查看楼梯、斜坡信息

## 安装与编译

安装系统依赖及 [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)。SDK 需要包含 DDS wrapper 头文件；CMake 会搜索 `/opt/unitree_robotics` 和标准安装前缀。

从 [MuJoCo 官方最新版下载页](https://github.com/google-deepmind/mujoco/releases/latest) 下载适合当前平台的发行包并解压。

```bash
git clone --single-branch --branch go2 https://github.com/wty-yy/unitree_mujoco.git unitree_mujoco_go2
cd unitree_mujoco_go2

sudo apt install cmake g++ libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev libfmt-dev

# 解压包含 simulate 源码的 MuJoCo 发行包，并创建mujoco-[版本号]的链接
ln -s /path/to/mujoco-[version] simulate/mujoco
cmake -S simulate -B simulate/build -DCMAKE_BUILD_TYPE=Release
cmake --build simulate/build -j$(nproc)
```

## 运行

默认配置必须使用手柄连接主机，通过 `./simulate/build/jstest` 测试手柄输入，否则需关闭手柄输入，才能启动仿真

```bash
./simulate/build/unitree_mujoco
./simulate/build/unitree_mujoco -s scene.xml
./simulate/build/unitree_mujoco -s obstacles.xml -i 0 -n lo
./simulate/build/unitree_mujoco --help
```

- `-s`、`--scene`：场景文件，覆盖 `robot_scene`，默认 `stairs.xml`；相对路径从 `unitree_robots/go2/` 解析，也支持绝对路径
- `-i`、`--domain_id`：DDS 域 ID，覆盖 `domain_id`，默认 `0`
- `-n`、`--network`：DDS 网络接口，覆盖 `interface`，默认 `lo`
- `-h`、`--help`：显示命令行帮助并退出

可用场景：`scene.xml`、`scene_terrain.xml`、`flat.xml`、`stairs.xml`、`cross_stairs.xml`、`cross_slope.xml`、`race_track.xml`、`racing.xml`、`four_platforms.xml`、`rough_flat.xml`、`wheel.xml` 和 `obstacles.xml`

更多配置通过 [`simulate/config.yaml`](simulate/config.yaml) 设置：

- `use_joystick`：`1` 启用手柄输入（默认），未连接手柄时设为 `0`
- `joystick_type`：手柄布局，可选 `xbox`（默认）或 `switch`
- `joystick_device`：手柄设备路径，默认 `/dev/input/js0`
- `joystick_bits`：手柄摇杆精度位数，默认 `16`
- `print_scene_information`：`1` 输出连杆、关节、执行器和传感器信息（默认），`0` 关闭
- `show_base_velocity`：`1` 显示机身 x/y 速度与偏航角速度（默认），`0` 关闭

## 代码框架

```text
unitree_mujoco/
├── simulate/                    # C++ 仿真器
│   ├── CMakeLists.txt           # 构建配置
│   ├── config.yaml              # 仿真配置
│   ├── mujoco/                  # MuJoCo 安装目录的软链接
│   └── src/                     # 仿真、DDS 桥接与手柄支持源码
└── unitree_robots/
    └── go2/                     # Go2 MJCF 模型与地形场景
```

## 许可证

本仓库原创代码采用 [MIT License](LICENSE)。源自 Unitree MuJoCo 的代码保留 BSD 3-Clause License，详见同一文件。第三方代码遵循各自的许可证。
