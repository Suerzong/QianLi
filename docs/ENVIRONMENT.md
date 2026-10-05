# ENVIRONMENT — 开发环境

> 记录 QianLi 开发环境检查结果、与目标平台的差异、缺失依赖与安装指引。
> 原则：缺少大型依赖时**不盲目安装**，先记录于此文档，再决定安装方案。

## 1. 目标平台（已定版）

| 项 | 值 |
|---|---|
| 操作系统 | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| 构建工具 | colcon、CMake（ROS 2 Jazzy 依赖 3.28） |
| 语言 | C++（GCC）、Python 3.12 |

> **2026-10-04 决策**：因开发虚拟机为 Ubuntu 24.04（"Ubuntu 24.04 ROS2 Jazzy"），项目目标平台由 Ubuntu 22.04 + Humble 调整为 **Ubuntu 24.04 + ROS 2 Jazzy**（见 [DEVLOG.md](DEVLOG.md)）。

## 2. 开发环境实况（2026-10-04 更新）

### 2.1 主机（Windows 11，开发编辑机）

| 检查项 | 结果 |
|---|---|
| 操作系统 | Windows 11 家庭版 中文版（10.0.26100，64 位） |
| 主机名 / 用户 | SUERZONG / sez18，HOME = C:\Users\sez18 |
| 磁盘 | C: 剩余 59.1 GB；D: 剩余 198.9 GB；E: 剩余 46.3 GB |
| Git | 2.53.0.windows.2（E:\Git\cmd\git.exe），身份已配置 |
| Python | 3.13.12（miniconda）+ 3.14.5（py） | 
| CMake | 4.3.1（Windows 侧其他工程用） |
| IDE | VS Code（E:\Applications\Microsoft VS Code）；CLion 2026.2.1 |
| 嵌入式工具链 | STM32CubeIDE/MX/Programmer/CLT、Keil5、Arduino、OpenOCD |
| 虚拟化 | VMware Workstation（运行 1 台虚拟机，见 §2.2） |

> 主机不参与 ROS 2 构建；ROS 2 工具链全部在虚拟机内运行。

### 2.1.1 SSH 快速连接（必读）

```bash
# 主机（Windows）免密登录，别名已配好
ssh qianli-vm

# 或一键脚本
bash scripts/tools/vm_ssh.sh

# 健康检查
bash scripts/tools/vm_check.sh
```

> 完整连接信息、排查清单与别名配置块见 **[SSH.md](SSH.md)**。
> 连不上时按 SSH.md §4 排查（虚拟机开机 → 端口 22 → sshd → 免密 → 别名）。

### 2.2 开发虚拟机（Ubuntu 24.04 + ROS 2 Jazzy）

| 检查项 | 结果 |
|---|---|
| VM 名称 | Ubuntu 24.04 ROS2 Jazzy（`D:\Ubuntu-VM\ubuntu24-ros2.vmx`） |
| 系统 | Ubuntu 24.04.4 LTS（Noble Numbat），内核 7.0.0-34-generic |
| 网络 | VMware NAT：**192.168.26.128**（hostname `ros2-ubuntu`，MAC 00:0c:29:bd:0b:9c） |
| SSH | 22 端口开放；用户 `ros`；**免密已配好**（别名 `qianli-vm`），连接指南见 [SSH.md](SSH.md) |
| sshd 自启 | ✅ enabled + active（2026-10-04 复核） |
| ROS 2 | ✅ `/opt/ros/jazzy`，`.bashrc` 已自动 source |
| colcon | ✅ /usr/bin/colcon |
| Python | ✅ 3.12.3（系统） |
| CMake | ✅ 3.28.3 |
| Git | ✅ 2.43.0 |
| 磁盘 | / 剩余约 **12 GB**（84% 已用）——注意空间，构建产物及时清理 |
| GPU | ❌ 无（VMware 未直通，`nvidia-smi` 不存在）→ 深度学习训练需另配 GPU 云服务器 |
| 内存 | 5.8 GiB 总 / 约 3.5 GiB 可用 |
| 关键包 | ✅ urdf / xacro / rviz2 / robot-state-publisher / joint-state-publisher / ros2-control / moveit |
| 待装包 | ⏳ nav2-bringup / robot-localization / gazebo-ros-pkgs（后续 Milestone 需要时再装） |
| 工作区 | `~/QianLi`（与主机仓库同步）；`~/arm_ws`、`~/arm-final` 已不存在（2026-10-04 复核，模型已整合进 qianli_description） |

## 3. 构建验证状态

- ✅ **Milestone 0 已在虚拟机内验证**：`colcon build --symlink-install` 通过（见 [DEVLOG.md](DEVLOG.md)）；
- 构建命令：`source /opt/ros/jazzy/setup.bash && cd qianli_ws && colcon build --symlink-install`；
- 辅助脚本：`bash scripts/tools/build.sh`（自动定位 qianli_ws）。

## 4. 缺失依赖清单（按需安装，不预装）

| 依赖 | 目标环境 | 安装方式 | 引入时机 |
|---|---|---|---|
| nav2 全套 | Ubuntu 24.04 | `sudo apt install ros-jazzy-nav2-*`（按需子集） | 并行路线（虚拟底盘） |
| robot_localization | Ubuntu 24.04 | `sudo apt install ros-jazzy-robot-localization` | 并行路线 |
| gazebo / 仿真 | Ubuntu 24.04 | `sudo apt install ros-jazzy-gazebo-ros-pkgs` | 并行路线 |
| MoveIt2 | ✅ 已装（ros-jazzy-moveit） | — | Milestone 3 |
| 3D LiDAR 驱动 | 真实硬件到位后 | 按厂商 SDK | 真实 LiDAR 到位 |

> 新依赖加入前，先在本文档登记，再安装。

## 5. 本机/虚拟机环境注意事项

- 主机 conda Python（3.13/3.14）**不要**用于 ROS 工具；虚拟机内使用系统 Python 3.12；
- 虚拟机磁盘紧张（剩 13 GB），构建后及时 `bash scripts/tools/clean.sh` 清理；
- 虚拟机无 VMware hgfs 共享文件夹，仓库通过 **scp / git** 同步；
- D:\projects 下已有其他项目（如 TALOS26 秋季招新考核题），QianLi 独立 Git 仓库，互不影响；
- 虚拟机内已有机械臂相关工作区（~/arm_ws 等），与本项目隔离。

## 6. 更新记录

- 2026-10-04：首次环境扫描（Windows 11 主机）；随后接入 VMware 虚拟机（Ubuntu 24.04 + Jazzy），目标平台定版为 24.04/Jazzy，Milestone 0 构建验证通过。
