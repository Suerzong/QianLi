# QianLi

千里之行，始于足下。

An autonomous mobile manipulation robot for indoor exploration, semantic navigation and embodied task execution.

## 项目简介

QianLi（千里）是一个面向**室内复杂环境**的自主探索、语义认知与任务执行移动机器人项目。

项目最终目标：

- 在未知室内环境中自主探索（不依赖人工提前建图），利用 SLAM 构建环境几何地图；
- 自动寻找未探索区域并持续扩展地图，自主定位、规划路径、动态避障；
- 视觉识别门、房间、走廊、电梯、自动售货机等语义目标，并与空间坐标关联建立 Semantic Map；
- 接收自然语言指令（如"去自动售货机帮我拿一瓶水"），转换为结构化任务并自主执行；
- 使用机械臂完成抓取、按按钮等操作；
- 长期保存已探索的地图、语义与环境知识，使机器人逐渐"认识"所在建筑。

系统定位：

- **Autonomous Mobile Manipulator** + **Semantic Navigation** + **Embodied Agent**

中文描述：面向室内复杂环境的自主探索与语义导航移动机器人。

## 当前阶段

**QianLi Simulation v0.3**：四全向轮 Omni X-drive 的 ros2_control、Gazebo 理想运动、虚拟 IMU/LiDAR、SLAM 和 Nav2 软件原型。真实机械臂开发路线继续保留。

## 目标平台与技术栈

| 项 | 内容 |
|---|---|
| 操作系统 | Ubuntu 24.04 |
| ROS 版本 | ROS 2 Jazzy |
| 主要语言 | C++ / Python |
| 核心框架 | ROS2、RViz2、URDF/Xacro、TF2、ros2_control、MoveIt2、Gazebo、Nav2、robot_localization |
| 未来 | FAST-LIO2/LIO-SAM、Frontier Exploration、YOLO、AprilTag、OCR、Semantic Mapping、VLM、LLM Agent、Behavior Tree、Multi-floor Navigation |

> 注意：当前开发机为 Windows 11，ROS 2 工具链运行在 VMware 虚拟机（Ubuntu 24.04 + ROS 2 Jazzy）中，详见 [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md)。

## 目录结构

```
QianLi/
├── docs/          # 项目文档（PROJECT / ARCHITECTURE / ROADMAP / ENVIRONMENT / HARDWARE / DEVLOG）
├── hardware/      # 硬件资料（机械臂 / 底盘 / 传感器 / 电子 / CAD）
├── firmware/      # 固件（STM32 底层控制器）
├── simulation/    # Gazebo 仿真（worlds / models / configs）
├── datasets/      # 数据集（不纳入版本控制）
├── scripts/       # 辅助脚本（setup / tools）
└── qianli_ws/     # ROS 2 工作区（colcon）
    └── src/       # ROS 2 packages
```

## 快速开始

工作区基于 **Ubuntu 24.04 + ROS 2 Jazzy**（虚拟机 `192.168.26.128`，用户 `ros`）：

```bash
# 1. 在 Ubuntu 24.04 上安装 ROS 2 Jazzy（基础环境，一般已装好）
bash scripts/setup/install_ros2_jazzy.sh

# 2. 构建工作区
source /opt/ros/jazzy/setup.bash
bash scripts/tools/build.sh            # 等价于: cd qianli_ws && colcon build --symlink-install

# 3. 加载工作区环境
source scripts/setup/source_env.sh
```

## Simulation v0.3 复现

工程仍使用原有 `/home/ros/QianLi/qianli_ws`，没有重复工作区。底盘为 **Omni X-drive**。
依赖：ROS 2 Jazzy、Gazebo Harmonic / ros_gz、gz_ros2_control、官方 omni_wheel_drive_controller、
slam_toolbox、navigation2、nav2_bringup、RViz2、Xacro；本次已获授权安装缺少的 SLAM/Nav2 包。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ros/QianLi/qianli_ws
colcon build --symlink-install
source install/setup.bash
# 与现有真实机械臂隔离；同一仿真的所有终端必须使用相同 Domain/Partition。
export ROS_DOMAIN_ID=73 GZ_PARTITION=qianli_v03
ros2 launch qianli_bringup sim.launch.py slam:=true nav2:=true rviz:=true
```

`slam:=false nav2:=false` 只运行底盘、传感器和桥接；`slam:=true nav2:=false` 建图；
`slam:=false nav2:=true` 用保存地图和 AMCL OmniMotionModel 定位导航。
`map:=/absolute/path/map.yaml` 可替换保存地图；默认 `qianli_slam/maps/qianli_test_map.yaml`。
`gui:=false` 关闭 Gazebo GUI，独立 `rviz` 参数仍生效。仿真模式固定为已验证的 `ideal_kinematic_sim`。

VM 软件渲染：桌面终端保留自身 DISPLAY。无桌面时可使用已安装的 Xvfb：

```bash
Xvfb :98 -screen 0 1500x950x24 &  # 若 :98 已运行，直接复用
export DISPLAY=:98 LIBGL_ALWAYS_SOFTWARE=1
ros2 launch qianli_bringup sim.launch.py gui:=false slam:=true nav2:=true rviz:=true
```

不要同时启动 mock/control.launch 和 Gazebo 的 controller_manager，也不要在同一 Domain
运行两个整套仿真。`gui:=false` 默认使用 GLX；本 VM 的 EGL headless rendering 激光为 infinity，未采用。

第二个终端 source 相同环境并设置相同 ROS_DOMAIN_ID：

```bash
ros2 run qianli_control teleop.py --ros-args -p use_sim_time:=true
# W/S 前后，A/D 左右，Q/E 旋转，空格停止，X 退出；键重复停止后自动归零。
ros2 run qianli_control test_xdrive_kinematics.py  # 几何/公式独立检查
# --live 模式仅在独立 mock GenericSystem 会话检查实际控制器输出。
ros2 run qianli_control motion_test.py --gazebo --ros-args -p use_sim_time:=true
ros2 run qianli_sim check_imu_motion.py --ros-args -p use_sim_time:=true
cd /home/ros/QianLi
./scripts/test_base_sim.sh --map --navigation
ros2 run qianli_navigation navigation_test.py --ros-args -p use_sim_time:=true
```

自动运动/IMU 测试会发命令，请在 Nav2 没有活跃目标时运行。Nav2 测试依次发三个 NavigateToPose
目标，独立检查 Gazebo 位姿与场景障碍，最后发送零速度。RViz 顶部 **Nav2 Goal** 支持鼠标拖动目标朝向。

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/my_qianli_map --ros-args \
  -p use_sim_time:=true -p map_subscribe_transient_local:=true -p save_map_timeout:=20.0
# 保存地图模式的自动系统检查：
./scripts/test_base_sim.sh --map --navigation --localization amcl
```

几何：底板 0.70 × 0.70 × 0.004 m、切角 0.15 m、长边 0.40 m、斜边 0.212132 m；
base_link 高 0.09 m；轮 r=0.075 m、宽 0.038 m、中心 (±0.294,±0.294,0.075) m。
控制器 R=0.4157787873 m，wheel_offset=π/4，CCW 顺序 FL/RL/RR/FR；+axis 径向朝外。
Nav2 使用真实八边形与 0.06 m padding（包含轮子外探的保守余量），vx/vy 均允许正负。

**仿真边界**：Gazebo 机器人接触与重力禁用，由官方控制器 `/odom` twist 通过内部理想执行接口驱动。
里程计是 **commanded/open-loop**，不是 measured wheel odometry。轮 joint states 来自 Gazebo，
激光来自实际 Gazebo 场景，IMU 来自 Gazebo 传感器；独立 ground truth 不发布 TF。
这不能验证轮子摩擦、打滑、力矩、惯性或碰撞停止。当前几何圆柱没有伪装成真实 omni 滚子动力学。
`wheel_physics_sim`、真实质量/重心/惯量、实机电机/编码器方向与 STM32 interface 均为后续工作。

当前自动验收日志位于 `qianli_ws/log/sim_v03`（Git 忽略），源码与地图随阶段 Git commits 保存。

## 文档索引

| 文档 | 内容 |
|---|---|
| [docs/PROJECT.md](docs/PROJECT.md) | 项目定义、目标能力、技术栈 |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 软件架构、模块职责、TF / 地图 / Agent 架构 |
| [docs/ROADMAP.md](docs/ROADMAP.md) | 开发路线与 Milestone 定义 |
| [docs/ENVIRONMENT.md](docs/ENVIRONMENT.md) | 开发环境检查结果与环境差异 |
| [docs/HARDWARE.md](docs/HARDWARE.md) | 硬件清单、接口规划、缺失项 |
| [docs/DEVLOG.md](docs/DEVLOG.md) | 开发日志 |

## 许可证

[MIT](LICENSE)
