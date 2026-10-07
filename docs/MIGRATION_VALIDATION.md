# Ubuntu 22.04 迁移验证记录

日期：2026-10-07。实施步骤见 [原生迁移与验收](UBUNTU22_MIGRATION.md)。已经另行安装持久 Ubuntu 22.04/Humble 虚拟机，运行入口见 [VM 使用说明](VM_HUMBLE.md)。笔记本尚未安装原生 Ubuntu，原生 Linux GPU及真机运动/抓取验收仍未完成。

## 持久 VM 与生产 CUDA 环境

新 VM 位于 `D:\VMs\QianLi-Ubuntu22-Humble\qianli-humble.vmx`，项目 `/home/ros/QianLi`，Git 分支 `codex/ubuntu22-humble`。采用校验过的 Canonical Jammy cloud VMDK，独立 120GB 磁盘、8GB 内存、8 vCPU。已安装 Ubuntu 桌面、HWE 6.8.0-138 内核、Humble、固定 ROS/训练环境和桌面入口；两次冷启动均自动登录并打开模拟 RViz，RobotModel/Global Status 显示 OK。没有修改或关闭另外两台运行中的 VM。

| 实际环境 | 验收结果 |
|---|---|
| 独立 Ubuntu 22.04.5 VM / Humble / Python 3.10 | 六包构建成功；colcon 23 项测试全部通过；ROS 安装模块、视觉离线自检、模拟关节/TF/闸门消息通过；`allow_motion=false`、`calibrated=false` |
| 同一 VM 的隔离训练环境 | 原 17 项迁移回归加 6 项只读报告/外参接口回归，共 23 项通过；20mm/40mm 各两个 spawn 进程、1024 步 PPO、权重更新与模型保存/加载通过；EGL 128×128 渲染通过 |
| Windows 生产 `.venv-train-win` / RTX 5070 Ti / 591.74 / Torch 2.8.0+cu128 | 20mm/40mm 均通过实际 CUDA 运算、1024 步更新、保存/加载；25 项迁移/协议测试通过，1 项 Linux sysfs 用例在 Windows 跳过、在 VM 通过 |
| VM CPU 与 Windows CUDA，原有两个策略归档 | `bc_policy.zip` 和 `ppo_bc_final.zip` 均可加载和预测；另存后重新加载的动作输出、全部策略权重与加载前一致；原归档未覆盖 |
| Windows 生产环境的完整 `train.py` | 40mm / 两环境 / 128 步 / BC 热启动成功，生成 64/128 步 checkpoint 与最终模型；短跑成功率 0%，不代表抓取能力提升 |
| 新 VM 的外置 UVC 相机 | xHCI + MJPG，640×480 连续 60 帧通过，实际图片无损坏条带；已配置稳定 by-id 路径；未执行新的几何标定 |
| 新 VM 的 CH343 适配器 | `/dev/qianli_arm` 与 by-id 路径、dialout 权限、实际 sysfs USB 接口发现通过；上电后六个位置和扭矩均可读取，扭矩均为 0；部分手摆关节超出软限位，姿态检查未通过 |
| 新 VM 的 direct 模式 | 实际 `/joint_states`、TF 和闸门状态已收到；`allow_motion=false`、`calibrated=false`，运动使能请求返回拒绝；无目标位置或 EEPROM 写入；完整证据见 `migration_assets/vm-acceptance/hardware/` |

ROS 在线 GitHub 下载在 guest 内发生 TLS 断开，安装已通过宿主机下载的官方数据完成：`ros2-apt-source` 包匹配发布方 SHA256，rosdep YAML 绑定具体 rosdistro commit，官方 Humble 缓存来源由官方 index 解析，guest 内逐文件校验并执行了新的 `rosdep update`。此持久 VM 没有使用旧 VM 的 rosdep 缓存。`ament_python` 是 colcon build type，不是可解析的 ROS 包依赖；清除了三个 Python 包中的错误 buildtool 声明，保留 build type。

验收脚本现在等待 DDS 发现并指定订阅消息类型，避免默认一秒发现窗口造成的误失败。相机最初 YUYV 超时、EHCI/MJPG 出现图像条带；最终换为 xHCI 并在机器配置中指定 MJPG，经过连续采集和图像检查后才记录通过。短桌面检查结束后保留输出窗口。

持久 VM 的完整证据在 `migration_assets/vm-install/`、`migration_assets/vm-acceptance/`；Windows 生产日志在 `migration_assets/production-gpu-20mm.log`、`production-gpu-40mm.log`、`production-pretrain.log`。下文保留此前隔离验证记录，便于比较验证范围。

补充的 6 项回归检查只读工具在超限、缺少舵机回应或扭矩开启时拒绝报告全部通过，以及合成外参写入实际拟合的 `grid_origin_z`，供严格外参加载器读取。VM 的训练环境完整 23 项通过，ROS 环境新增 6 项通过；Windows 完整回归为 22 项通过、1 项 Linux sysfs 用例跳过。

旧触点 `touch_marks_20261006.json` 与 `board_cam.npz` 已在独立目录重新回算：触点 RMS 4.67 mm、留一平移抖动 17.51 mm / 旋转 3.347°，未通过原有质量门槛，输出 `quality_ok=0`，严格加载器拒绝使用。日志在 `migration_assets/calibration-audit/`；没有安装到实际 `calib/`。旧关节限位可复用，不代表这些几何外参已经合格。

旧 VM 原配置的最终只读验收记录在 `migration_assets/vm-acceptance/hardware/final-readonly/result.json`：六个舵机通信正常，前后扭矩均为 0，六个动态及两个静态 TF 均收到（含 `tcp_link`），使能请求明确拒绝；关闭 IK 后未发布候选运动指令。肩升降 788 / 下限 806，肘关节 4076 / 上限 4064，当前手摆姿态仍超旧软限位，因此 `hardware_ready=false`，没有验收运动或抓取。

## 已交付

- 补齐完整 `so101_bringup` ROS 包、公开模块兼容入口、launch、模型、配置和原驱动测试。工作区现在有六个可构建包。
- 提供 Humble 安装、两个 Python 3.10 环境、固定依赖、构建、设备规则、资产校验与恢复、模拟 ROS 验收和 CUDA 训练短跑脚本。
- 资源从项目位置、ROS 安装或显式覆盖解析；标定写入持久目录，串口/相机接受稳定设备路径，USB 看门狗发现实际接口。
- 修复未知棋盘导致的仿真初始化错误，以及 MuJoCo 不能直接读取 ROS `package://` 网格的问题。真机抓取继续要求质量合格、有限数值的外参。
- 训练分别记录 20mm/40mm 场景，检查 CUDA 运算、多个工作进程、模型更新及保存/加载；输出目录拒绝混用不同场景。
- 添加 `.github/workflows/ubuntu22-humble.yml`。本次没有推送或触发 GitHub Actions。

## 实际验证

| 环境 | 执行内容 | 结果与边界 |
|---|---|---|
| Ubuntu 22.04.5 用户空间、Humble、Python 3.10 | 六包 colcon 构建；驱动/IK/协议测试 | 构建成功；23 项测试，0 错误、0 失败、0 跳过 |
| 同一 Humble 环境 | 安装模块导入、视觉像素管线/颜色检测自检 | 全部通过；包从实际安装路径加载 |
| 同一 Humble 环境 | 17 项迁移回归 | 全部通过，含 Linux USB 接口发现、归档校验和标定拒绝；历史/ROS 两种模型 × 20mm/40mm × 有/无棋盘，共 8 个固定种子仿真组合 |
| 同一 Humble 环境 | 模拟驱动启动、`allow_motion=false`、关节状态、TF、安全闸门消息 | 通过；专用 ROS domain 82，没有启动 direct 驱动或发送真机指令 |
| Windows、独立 Python 3.10、全部固定版本 | 迁移与协议回归 | 25 项通过；Linux sysfs 用例 1 项跳过，已在 Linux 单独通过 |
| Windows、RTX 5070 Ti Laptop、Torch 2.8.0+cu128 | 20mm/40mm 各两个 spawn 工作进程、1024 步 PPO | 两个场景均通过实际 CUDA 矩阵运算、权重更新和模型保存/加载一致性；尚不能据此确认 Linux GPU 驱动 |
| 同一 Windows 固定环境 | 完整 `train.py` 入口，40mm、两个环境、128 步 CUDA 训练 | 成功生成场景元数据、64/128 步 checkpoint、最终策略和训练日志；短跑评估成功率为 0%，不代表抓取能力提升 |
| Windows 固定环境 | MuJoCo 128×128 离屏渲染、视觉离线自检、pip check | 通过；原生 Linux EGL 渲染仍待检查 |
| 同一 Windows 固定环境 | 六个历史场景构造器载入 ROS URDF、查看器导入；四个旧基本体夹爪场景省略未知棋盘 | 通过；这部分补充检查在 Windows 执行 |
| 同一 Windows 固定环境 | 尝试把 20mm 运行写入 40mm 输出目录，或复用无场景元数据的历史目录 | 两种情况均拒绝；历史文件校验值保持一致 |
| 旧 Ubuntu 24.04/Jazzy VM，原有训练环境 | 两个尺寸的多进程 CPU 训练短跑 | 通过；该环境的 Torch 版本不同，仅作 Linux 多进程补充验证 |

Humble 验证使用官方 Ubuntu Base 22.04.5 amd64 镜像，在旧 Ubuntu 24.04 VM 内创建隔离 chroot。镜像 SHA256 已校验；安装了真实 Humble 和固定 ROS Python 依赖，并为完整仿真回归额外安装 Gymnasium 1.4.0。该环境共享 VM 内核，没有挂载机械臂、相机或 GPU。rosdep 在线更新遭遇 GitHub 网络故障，验证采用旧 VM 的已有 rosdep 缓存；原生安装仍需完成在线 rosdep 更新。

最终完整日志显示 `17 passed`、`23 tests, 0 errors, 0 failures, 0 skipped` 和模拟 ROS 验收通过。日志中的 ikpy 固定关节 axis/active mask 警告来自现有 URDF，未引起测试失败。验证完成后已归档日志、停止隔离环境进程并删除临时 rootfs，释放约 4.8GB；旧 VM 工程和资产保留。

## 本地证据与资产

以下路径相对于项目根目录，机器资产被 Git 忽略，应随迁移单独复制。

| 文件 | 内容 |
|---|---|
| `.migration-backups/20261007-172250/workspace.zip`、`manifest.json` | 实施前 545 个项目文件、原始 Git 状态与逐文件 SHA256；保全未提交和未跟踪工作 |
| `migration_assets/live-vm-20261007-full.zip` | 从重启后的旧 VM 取回的 173 个驱动、有效配置、持久标定、回滚记录和网格文件 |
| `migration_assets/current-project-assets-20261007.zip` | 当前驱动、ROS/历史模型、配置和可用标定的迁移资产包；代码工作区须另行复制 |
| `qianli_ws/src/qianli_arm/migration_provenance.json` | 驱动原始文件校验值、模型来源、VM 核对及限位合并记录 |
| `migration_assets/humble-validation-20261007.zip` | 12 个日志/证据文件，含构建、最终 17 项回归、colcon XML、ROS 状态/TF/闸门与依赖解析记录；内部逐文件 SHA256 已复核 |
| `migration_assets/gpu-smoke-20mm.json`、`gpu-smoke-40mm.json` | 两个尺寸的固定版本 CUDA 短跑结果 |
| `migration_assets/trainer-check/native_40mm/` | 完整训练入口的元数据、策略、checkpoint 和日志 |
| `.venv-migration-check/resolved-requirements.txt` | 本次 Windows Python 3.10 验证环境的实际解析版本 |

Humble 证据 ZIP 的 SHA256：`f7c9b253718abd8c837e1d828b692802a89e5b96beda4c132dc71f30dd63473f`。

旧 VM 和本地原始限位均已保留。迁移默认参数采用两者交集，只将本地 `shoulder_pan` 上限进一步收紧至 3273；零位与方向不变，没有写入舵机 EEPROM。旧 VM 的实际运行参数另存为 `config/driver_params.previous_vm.yaml`，可通过 `QI_DRIVER_CONFIG` 选择；其窗口比交集更宽，不必因操作系统迁移重新标定限位。

## 尚待原生机器完成

- 安装 Ubuntu 22.04.5/HWE 和支持 Blackwell 的 NVIDIA open 驱动，验收显示、网络、休眠、Secure Boot 和实际 Linux CUDA 运算。
- 原生安装后按迁移文档重建两个环境并完成 rosdep、构建/测试、两个尺寸训练短跑、Linux EGL 与 RViz/TF 检查；这些软件项已经在持久 VM 完成，原生硬件仍需验收。
- 原生系统重新核对相机、串口和 USB 接口；这些设备路径/权限、相机采集和 USB 发现已在持久 VM 验证，没有执行 USB 自动复位。
- 已恢复相机内参、`board_cam.npz` 和触点记录；**缺少合格 `extrinsic.txt`、`tcp_calib.txt` 等临时标定产物**，需要依据原生机器的实际几何重新验证或标定。
- 持久 VM 已在用户确认支撑后完成 direct 只读状态及使能拒绝检查；当前手摆姿态仍有超软限位关节。核对姿态与合格标定后，才继续现有低速运动、停止及抓取验收。原生系统仍需重复硬件验收。

只有以上硬件与标定检查全部通过，才切换日常开发环境。旧 VM 应继续保留。
