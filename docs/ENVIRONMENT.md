# ENVIRONMENT — 开发环境

> 记录 QianLi 开发环境检查结果、与目标平台的差异、缺失依赖与安装指引。
> 原则：缺少大型依赖时**不盲目安装**，先记录于此文档，再决定安装方案。

## 1. 目标平台

| 项 | 值 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| 构建工具 | colcon、CMake（ROS 2 Humble 依赖 3.22） |
| 语言 | C++（GCC）、Python 3.10 |

## 2. 当前开发机检查结果（2026-10-04）

| 检查项 | 结果 | 说明 |
|---|---|---|
| 操作系统 | Windows 11 家庭版 中文版（10.0.26100，64 位） | ⚠️ 与目标平台（Ubuntu 22.04）不一致 |
| 主机名 / 用户 | SUERZONG / sez18，HOME = C:\Users\sez18 | |
| 磁盘 | C: 剩余 59.1 GB；D: 剩余 198.9 GB；E: 剩余 46.3 GB | QianLi 位于 D:\projects（空间充足） |
| Git | 2.53.0.windows.2（E:\Git\cmd\git.exe） | ✅ 身份已配置：Suerzong / suerzong2007@gmail.com |
| ROS 2 | ❌ 未安装（无 ros2 命令、无 ROS_DISTRO、无 /opt/ros 或 C:\ros 等路径） | 见 §3 |
| colcon | ❌ 未安装（无 colcon 命令） | 随 ROS 2 环境安装 |
| Python | 3.13.12（miniconda3，E:\Applications\miniconda3）；py 启动器另见 3.14.5 | ⚠️ 本机 conda 环境，与 Ubuntu 侧系统 Python 3.10 无关 |
| pip | 26.0.1（miniconda） | |
| CMake | 4.3.1 | ⚠️ 远高于 ROS 2 Humble 的 3.22；仅用于 Windows 侧其他工程 |
| WSL | wsl.exe 存在但**未注册任何发行版**；E:\Applications\WSL 下有 Ubuntu-24.04 目录（未注册） | 见 §3 选项 B |
| VS Code | ✅ E:\Applications\Microsoft VS Code（code 命令可用） | 建议安装 ROS 2 扩展 |
| CLion | ✅ E:\Applications\JetBrains\CLion 2026.2.1 | |
| 嵌入式工具链 | STM32CubeIDE / STM32CubeMX / STM32CubeProgrammer / STM32CubeCLT / Keil5 / Arduino IDE / OpenOCD | ✅ 适合后续 firmware/stm32 开发 |
| 其他 | PCL、Docker Desktop、Ninja、Tailscale 等 | |

## 3. 关键差异：本机无 ROS 2 / colcon

当前开发机为 Windows 11，无法直接运行 ROS 2 Humble 工具链，因此：

- `colcon build` **无法在本机验证**（属已知环境限制，非错误）；
- 工作区结构、package 元数据（package.xml / CMakeLists.txt）已按 ROS 2 Humble 标准编写，待 Ubuntu 环境执行首次构建；
- 初始化过程**未安装任何软件、未修改系统全局配置**（PATH / shell 启动脚本等）。

**构建验证路径（三选一，需用户决策，未擅自执行）：**

| 选项 | 说明 | 备注 |
|---|---|---|
| A. 专用 Ubuntu 22.04 主机 / 双系统 | 与目标平台完全一致 | 推荐；安装脚本 scripts/setup/install_ros2_humble.sh |
| B. WSL2 + Ubuntu（本机有 Ubuntu-24.04 目录未注册） | 注册后可用，但发行版为 24.04，对应 **ROS 2 Jazzy**（非 Humble） | 若采用需统一 ROS 版本策略 |
| C. Docker（本机已装 Docker Desktop） | 可用 ros:humble 镜像快速验证构建 | 仅验证编译，RViz / Gazebo GUI 受限 |

## 4. 缺失依赖清单（不在本机安装）

| 依赖 | 目标环境 | 安装方式 |
|---|---|---|
| ROS 2 Humble | Ubuntu 22.04 | scripts/setup/install_ros2_humble.sh（基础：ros-humble-desktop + ros-dev-tools + colcon + rosdep） |
| Nav2 | Ubuntu 22.04 | 后续 Milestone 需要时在 Ubuntu 侧按需安装（ros-humble-nav2-*） |
| MoveIt2 | Ubuntu 22.04 | 后续 Milestone 3 时安装（ros-humble-moveit-*） |
| Gazebo | Ubuntu 22.04 | ros-humble-desktop 自带（ros-ign / gazebo） |

> 后续加入的大型依赖，先在本文档登记，再在 Ubuntu 环境安装，不污染当前开发机。

## 5. 本机环境注意事项

- Python 3.13 / 3.14 为 conda 环境，**不要**用于安装 ROS 工具（版本不兼容 ROS 2 Humble 的 Python 3.10 约束）；
- CMake 4.3.1 仅供 Windows 侧工程使用，不影响 Ubuntu 侧构建；
- D:\projects 下已有其他项目（如 TALOS26 秋季招新考核题），QianLi 为独立 Git 仓库，互不影响；
- 环境扫描与初始化全程非破坏性：未删除任何已有文件，未覆盖已有配置。

## 6. 更新记录

- 2026-10-04：首次环境扫描并记录（见 [DEVLOG.md](DEVLOG.md)）。
