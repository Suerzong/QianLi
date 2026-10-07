# Ubuntu 22.04 迁移验证记录

日期：2026-10-07。实施步骤见 [原生迁移与验收](UBUNTU22_MIGRATION.md)。这些结果验证仓库和运行基线；笔记本尚未安装原生 Ubuntu，也尚未完成原生 Linux GPU、相机、串口或真机验收。

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

旧 VM 和本地原始限位均已保留。迁移默认参数采用两者交集，只将本地 `shoulder_pan` 上限进一步收紧至 3273；零位与方向不变，没有写入舵机 EEPROM。

## 尚待原生机器完成

- 安装 Ubuntu 22.04.5/HWE 和支持 Blackwell 的 NVIDIA open 驱动，验收显示、网络、休眠、Secure Boot 和实际 Linux CUDA 运算。
- 按迁移文档创建两个环境并完成在线 rosdep、完整构建/测试；训练环境执行两个尺寸短跑和 Linux EGL 渲染，ROS 环境打开 RViz 核对模型与 TF。
- 实际相机采集、稳定串口路径/权限和 USB 看门狗验证；本次没有自动复位 USB。
- 已恢复相机内参、`board_cam.npz` 和触点记录；**缺少合格 `extrinsic.txt`、`tcp_calib.txt` 等临时标定产物**，需要依据原生机器的实际几何重新验证或标定。
- 支撑机械臂后按 `allow_motion=false` 检查 direct 状态、限位和闸门；注意 direct 初始化会关闭扭矩。之后才由操作者进行现有低速运动、停止及抓取验收。

只有以上硬件与标定检查全部通过，才切换日常开发环境。旧 VM 应继续保留。
