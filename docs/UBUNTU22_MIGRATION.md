# 原生 Ubuntu 22.04 迁移与验收

目标：当前 RTX 5070 Ti Laptop，Ubuntu 22.04.5 / HWE / ROS 2 Humble / Python 3.10。覆盖当前真机控制、视觉、MuJoCo 与 PPO 训练；Nav2、SLAM 等规划功能另行实现。

仓库已补齐完整 `so101_bringup` 包。公开的包名、节点入口、关节名及消息接口保留；默认仍为 `driver_mode=sim`、`allow_motion=false`。原生硬件验收通过之前保留旧虚拟机。

## 资产与版本

本次修改前的工作区快照：`.migration-backups/20261007-172250/workspace.zip`，逐文件 SHA256 和原始 Git 状态见同目录 `manifest.json`。包含当时未提交、未跟踪的项目文件；虚拟环境与历史构建目录不属于这个代码快照。

旧虚拟机重启后已重新连接。完整驱动、有效参数、持久标定、限位回滚记录及旧凸分解网格共 173 个文件保存于 `migration_assets/live-vm-20261007-full.zip`；该 ZIP 和恢复目录均为本地资产，不随 Git 推送。三份核心驱动源码与之前镜像一致，记录见 `qianli_ws/src/qianli_arm/migration_provenance.json`。

运行参数与本地限位存在差异。迁移默认窗口取两者交集：保留本地收紧，并将 `shoulder_pan` 的上限从 3276 收到运行版本的 3273。`zero_raw`、`direction` 未改变，未写舵机零位或 EEPROM。两个原始版本均保留；旧 VM 参数另存 `calib/driver_params.vm.yaml`，可显式用 `QI_DRIVER_CONFIG` 选择，但须先核对包络。

持久目录恢复了 `camera_intrinsics.yaml`、`board_cam.npz` 和触点记录。**当前未找到质量合格的 `extrinsic.txt`、`tcp_calib.txt` 等原来位于 `/tmp` 的文件。** 棋盘触点或旧 `config/board_frame.json` 不自动视为合格外参，需在原生系统按实际相机位置重新验证或标定。缺少合格外参时，视觉抓取入口拒绝使用；纯仿真省略未知棋盘。

以后在旧机器导出、在新机器恢复：

```bash
python3 scripts/tools/migration_assets.py export --output ~/qianli-assets.zip
# 将 ZIP 和整个当前工作区（包含未提交文件、config/）复制到新系统。
python3 scripts/tools/migration_assets.py verify ~/qianli-assets.zip
python3 scripts/tools/migration_assets.py restore ~/qianli-assets.zip \
  --destination migration_assets/recovered --calib-dir "$PWD/calib"
```

恢复先校验全部 SHA256，再检查所有目的路径；拒绝覆盖不同的现有文件。原始驱动和旧 `qianli_ws/config` 恢复到资产目录供比对，避免覆盖本地修复。新程序读取仓库 `config/` 中现有长期参数；新相机内外参及标定产物默认写 `calib/`，可用 `QI_CALIB_DIR` 改为独立持久目录。迁移时携带这两个目录和资产包，单独 `git clone` 不包含忽略的机器标定。

## 原生系统

备份个人数据和恢复介质后安装 Ubuntu 22.04.5。磁盘分区、格式化和启动项变更需要在本机安装阶段执行。采用 HWE 内核，并先确认有线/无线网络、显示、休眠唤醒和 Secure Boot 状态。

```bash
sudo apt update
sudo apt install linux-generic-hwe-22.04 ubuntu-drivers-common
ubuntu-drivers devices
```

Blackwell 使用 NVIDIA **open** 内核驱动。在 Ubuntu「附加驱动」中选择支持本 GPU 的较新 `*-open` 驱动，按提示完成 Secure Boot/MOK 注册并重启；不要将 Windows 驱动或闭源内核模块复制过来。

```bash
nvidia-smi
uname -r
lspci -k
ls -l /dev/serial/by-id/ /dev/v4l/by-id/
```

官方依据：[Ubuntu 生命周期](https://ubuntu.com/about/release-cycle)、[REP 2000](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst)、[NVIDIA 开源内核模块](https://developer.nvidia.com/blog/nvidia-transitions-fully-towards-open-source-gpu-kernel-modules/)。22.04 标准支持和 Humble 支持均至 2027 年 5 月，后续升级应提前安排。

## 安装与构建

从新的 Bash shell 执行，不继承 Conda 或 Jazzy。路径可任意选择：

```bash
cd ~/QianLi
bash scripts/setup/install_ros2_humble.sh
bash scripts/setup/setup_python_envs.sh
bash scripts/setup/install_device_rules.sh
# 注销、重新登录；重新插入串口适配器。
bash scripts/tools/build.sh
bash scripts/tools/validate_ubuntu22.sh
```

ROS 安装脚本在安装前严格核对 Ubuntu 22.04/amd64；先按 ROS 官方要求更新 systemd/udev，再安装 Humble。`build.sh` 用 ROS 虚拟环境的 Python 执行 colcon，确保节点入口使用 Python 3.10。

| 环境 | 用途 | 隔离方式 |
|---|---|---|
| `.venv-ros` | ROS、视觉、标定、安全闸门 | Python 3.10，继承系统 ROS 包 |
| `.venv-train` | MuJoCo、PPO/CUDA 训练 | Python 3.10，不继承系统包；从新 shell 加载 |

直接依赖固定为 NumPy 1.26.4、SciPy 1.15.3、OpenCV 4.10.0.84、MuJoCo 3.14.0、ikpy 4.1.0、PyYAML 6.0.2；训练增加 SB3 2.9.0、Gymnasium 1.4.0、PyTorch 2.8.0/cu128、TensorBoard 2.20.0。安装后写出各环境 `resolved-requirements.txt`，保存完整解析结果。Windows 虚拟环境必须重建。[PyTorch 官方安装组合](https://pytorch.org/get-started/previous-versions/#v280)

## 设备与资源配置

```bash
source scripts/setup/source_env.sh
export QI_ARM_PORT=/dev/qianli_arm
export QI_CAMERA=/dev/v4l/by-id/你的相机-video-index0
export QI_CALIB_DIR="$PWD/calib"
python3 scripts/tools/migration_check.py --target --ros --devices --calibration
```

`QI_CAMERA` 接受 OpenCV 数字索引或稳定 V4L2 路径，节点 `object_localizer` 也支持 `camera_device` 参数。串口规则按 CH343P 的 VID/PID 建立别名；若连接多个同型号适配器，应使用带序列号的 `/dev/serial/by-id/...`，不要依赖单一 VID/PID 别名。

`QI_PROJECT_ROOT` 指定项目根目录，`QI_SO101_PKG`、`QI_PARTS_DIR` 保留资源覆盖。显式指定的路径不存在时不悄悄退回其他机器的参数。USB 看门狗从配置串口发现实际 USB 总线、设备及接口，只对确认的 CH343P 操作；它需显式启动，不属于只读验收。

ROS 驱动/TF/安全闸门安装 `qianli_description` 中已有的 TCP 模型与网格。历史训练继续使用 `dual_twin/urdf`，这两个模型的臂关节变换一致，ROS 模型额外包含固定 `tcp_joint`。不要将模型覆盖变量从训练 shell 带入 ROS shell。

MuJoCo 载入 ROS 模型时会将 `package://` 网格地址解析为所选模型的实际文件；历史训练 URDF 的相对网格路径保持原有加载方式。仿真回归同时覆盖这两种布局。

## 仿真与训练验收

`validate_ubuntu22.sh` 检查目标系统、安装模块、视觉自检、colcon 测试、模拟驱动的关节状态、TF 和安全闸门；启动参数固定为 `sim/allow_motion=false`。GitHub 工作流提供同样的 Humble 构建回归，不包含物理硬件验收。

ROS 环境未安装 Gymnasium 时，pytest 会跳过仿真环境用例；必须继续执行下面训练环境的完整回归。CI 的测试环境额外安装 Gymnasium，以执行全部 17 项迁移用例。

从另一个新 shell 执行训练验收：

```bash
cd ~/QianLi
source scripts/setup/source_train.sh
python scripts/tools/train_smoke.py --device cuda --obj-size 0.02 --steps 1024
python scripts/tools/train_smoke.py --device cuda --obj-size 0.04 --steps 1024
MUJOCO_GL=egl python scripts/tools/migration_check.py --target --render
python -m pytest tests/test_migration.py -q
```

短跑检查实际 CUDA 矩阵运算、两个 `spawn` 工作进程、PPO 权重更新、模型保存及加载后输出一致性。CPU 物理仿真仍由 MuJoCo 执行。20mm 与 40mm 的固定种子回归分别记录，旧成功率不能归给当前 40mm 场景。

完整训练入口与旧 Windows 命令共享实现：

```bash
python dual_twin/scripts/train.py --device cuda --obj-size 0.04 \
  --steps 600000 --n-envs 10 --tag humble_40mm --pretrain dual_twin/rl_out/bc_policy.zip
```

每个训练输出目录保存 `scenario.json`，包括尺寸、种子、桌面、棋盘、资源和 Torch 版本；已有不同场景的目录会拒绝复用，无元数据的历史输出也要求选择新 `--tag`。这里只保证迁移和运行基线；40mm 成功率提升需独立实测。

## 真机分阶段验收

先支撑机械臂、清空工作范围，再使用只读模式：

```bash
source scripts/setup/source_env.sh
ros2 launch so101_bringup ik_demo.launch.py driver_mode:=direct \
  port:="$QI_ARM_PORT" allow_motion:=false calibrated:=false max_joint_speed:=0.2 use_ik:=false
```

检查 `/joint_states`、`/arm/status`、有效原始限位、TF 和安全闸门。`use_ik:=false` 在只读验收时关闭 IK 的候选指令发布；正常交互默认开启。**驱动的 direct 初始化会关闭扭矩**；只读表示拒绝运动使能，不能当作完全不写串口。机械臂须有支撑。通用验收脚本不会自动启动 direct，也不会运行舵机复位/重新定零工具。

旧 VM 的有效限位可直接复用，不需要因迁移操作系统重测。默认参数取旧 VM 与本地窗口的交集；要严格复现旧 VM，先设置 `QI_DRIVER_CONFIG="$PWD/qianli_ws/src/qianli_arm/config/driver_params.previous_vm.yaml"`。该配置来自已取回的旧 VM 实际运行参数，零位、方向和原有软限位均保留。只读查询成功而姿态超限时，应分别记录通信与姿态结果，不自动扩大软限位。

在合格标定、限位和停止行为通过后，再由操作者启动 `allow_motion:=true calibrated:=true max_joint_speed:=0.2`，按现有流程使能，验收小幅低速运动、停止和抓取。坐标预览通过后再发真机目标；参数外参覆盖必须同时设置 `extrinsic_quality_ok:=true`。迁移不运行 homing/EEPROM 写入脚本。

## 记录与回退

将 `qianli_ws/log/migration-*`、依赖解析结果、CUDA 短跑输出、相机实际帧尺寸、标定质量和低速真机结果存入验收记录。全部通过后切换日常环境。

旧 VM、原始资产包与工作区快照保留至验收完成。回退恢复代码/参数并使用旧 VM；不会通过“回退”修改舵机零位。不要使用通用 `clean.sh` 删除尚未归档的历史标定或限位日志。

当前已完成的验证与仍待完成的原生检查见 [迁移验证记录](MIGRATION_VALIDATION.md)。
