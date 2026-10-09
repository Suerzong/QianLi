# ENVIRONMENT — 开发环境

> 文档口径更新：2026-10-09。下述验收结果来自 2026-10-07 的记录；本分支 ROS 基线为 Ubuntu 22.04.5 / ROS 2 Humble / Python 3.10。各开发分支范围见 [README](../README.md)。

## 1. 当前运行环境与原生目标

| 环境 | 用途与状态 |
|---|---|
| 独立 Ubuntu 22.04.5 / Humble VM | 已安装桌面、HWE 6.8.0-138、Humble；运行 ROS、视觉、MuJoCo 仿真与 CPU 训练 |
| Windows 11 宿主机 | 编辑与 Git；独立 Python 3.10 / PyTorch 2.8.0+cu128 环境使用 RTX 5070 Ti Laptop 训练，实际 CUDA 运算已验证 |
| 笔记本原生 Ubuntu 22.04.5 / HWE / Humble | 后续部署目标；尚未安装，Linux GPU、显示、网络及原生设备验收待完成 |
| 旧 Ubuntu 24.04 / ROS 2 Jazzy VM | 保留资产与回退环境，不作为当前 Humble 构建基线 |

VM 使用 VMware 虚拟显卡，CPU 训练通过不代表 Linux NVIDIA 驱动或 CUDA 已验收。ROS 与训练环境分别隔离；Windows 虚拟环境不能复制到 Linux 使用。

## 2. 已安装的 Humble VM

| 项 | 值 |
|---|---|
| VM 文件 | `D:\VMs\QianLi-Ubuntu22-Humble\qianli-humble.vmx` |
| 系统 / ROS | Ubuntu 22.04.5 LTS（Jammy）/ `/opt/ros/humble` |
| Python | 系统 Python 3.10；ROS `.venv-ros`、训练 `.venv-train` |
| 资源 | 独立 120GB 磁盘、8GB 内存、8 vCPU |
| 网络 / SSH | VMware NAT `192.168.26.22`；用户 `ros`；别名 `qianli-humble`，SSH 使用密钥 |
| 项目 / 分支 | `/home/ros/QianLi` / `codex/ubuntu22-humble` |
| 桌面 | 自动登录，模拟 RViz 自动启动；按用户要求关闭客体自动锁屏与休眠 |
| 机械臂串口 | `/dev/qianli_arm`，CH343P；多个同型号设备应使用带序列号的 by-id 路径 |
| 外置相机 | `/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0`；xHCI、MJPG、640×480 |
| 持久机器配置 | `~/.config/qianli/environment.sh`；标定默认存于项目 `calib/` |

Windows 桌面入口 **QianLi - Ubuntu22 Humble** 可打开 VM。SSH 与排查命令见 [SSH.md](SSH.md)，桌面、训练和设备入口见 [VM_HUMBLE.md](VM_HUMBLE.md)。IP 属于本机安装记录，复制到其他机器后应重新核对。

## 3. 安装与日常构建

新的 Ubuntu 22.04 环境按 [UBUNTU22_MIGRATION.md](UBUNTU22_MIGRATION.md) 安装 ROS、固定 Python 依赖和设备权限。已有环境的常用命令：

```bash
cd ~/QianLi
bash scripts/tools/build.sh
source scripts/setup/source_env.sh
bash scripts/tools/validate_ubuntu22.sh
```

`source_env.sh` 默认加载 Humble，拒绝混入已经加载 Jazzy 的 shell。使用新的 Bash shell，并由项目脚本加载 `.venv-ros`；宿主机 Conda Python 3.13/3.14 不用于 ROS 构建。

从另一个新 shell 使用独立训练环境：

```bash
cd ~/QianLi
source scripts/setup/source_train.sh
python scripts/tools/train_smoke.py --device cpu --obj-size 0.04 --steps 1024
```

当前 VM 使用 `--device cpu`。宿主机 GPU 命令见 [VM_HUMBLE.md](VM_HUMBLE.md)；原生 Ubuntu 安装后须独立验证 Linux CUDA，再执行 `--device cuda`。

## 4. 验收结论与剩余项

| 范围 | 2026-10-07 的验证结论 |
|---|---|
| Humble 工作区 | 6 包构建成功；23 项 colcon 驱动/IK/协议测试通过 |
| 软件回归 | VM 独立训练环境完整 36 项迁移回归通过；视觉自检、RViz、模拟 joint_states / TF / 安全闸门通过 |
| 仿真 / 训练 | 20mm 与 40mm 分别完成两个 spawn 进程、1024 步 PPO、模型更新及保存/加载；VM EGL 渲染通过 |
| 宿主机 CUDA | 两种尺寸短跑与实际 CUDA 运算通过；40mm 完整入口的 128 步短跑成功率为 0%，不代表抓取能力提升 |
| VM 设备接入 | 相机连续 60 帧通过；六舵机位置/扭矩可读、扭矩均为 0；direct 状态/TF 与使能拒绝通过 |
| VM 真机运动 / 抓取 | 未验收；最后手摆姿态部分关节超旧软限位，缺少合格几何外参，现场补采待进行 |
| 原生部署 | 未验收；安装、Linux GPU、设备及完整软件/硬件回归均待执行 |

完整证据路径、测试统计口径和真机限制见 [MIGRATION_VALIDATION.md](MIGRATION_VALIDATION.md)。历史抓取成绩与迁移验收分别记录，20mm 的旧成绩不能归到当前 40mm 场景。

## 5. 后续依赖与旧环境

当前迁移覆盖本分支已有控制、视觉、MuJoCo 与训练实现。[移动仿真分支](https://github.com/Suerzong/QianLi/tree/codex/cloud-model-training) 已有 Jazzy/Gazebo Harmonic 的 Omni、SLAM、Nav2 与 Frontier 原型；它们尚未迁入或通过本分支 Humble 验收。后续集成依赖需按 Humble 选择并验证，不沿用 `ros-jazzy-*` 安装命令。MoveIt2 规划集成另按里程碑推进。

旧 VM：`D:\Ubuntu-VM\ubuntu24-ros2.vmx`，Ubuntu 24.04/Jazzy，SSH 别名 `qianli-vm`，此前地址 `192.168.26.128`。`vm_ssh.sh` / `vm_check.sh` 是该旧 VM 的历史脚本。旧资产与环境扫描保留在 [DEVLOG.md](DEVLOG.md)，使用前核对实际运行状态。

Ubuntu 22.04 标准安全维护与 Humble 支持均至 **2027 年 5 月**；Ubuntu 的延长安全维护是另一种覆盖，不能视为 Humble 支持延期。[Ubuntu 生命周期](https://ubuntu.com/about/release-cycle)、[REP 2000](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst)。

## 6. 更新记录

- 2026-10-09：统一当前 22.04/Humble 基线、VM 与宿主机 CUDA 分工，以及原生部署待验收边界。
- 2026-10-07：安装独立 Humble VM 和 Windows CUDA 训练环境，完成上述软件与只读硬件检查。
- 2026-10-04：首次在旧 Ubuntu 24.04/Jazzy VM 构建；当时的环境决定保留为历史记录。
