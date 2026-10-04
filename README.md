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

**Phase 0 — Foundation**：建立可维护、可扩展、可正常 `colcon build` 的 QianLi ROS 2 工作区。

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
