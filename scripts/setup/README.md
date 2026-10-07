# 环境安装脚本

默认目标是原生 Ubuntu 22.04.5 + ROS 2 Humble，Python 3.10。

| 脚本 | 用途 |
|---|---|
| install_ros2_humble.sh | 校验系统后安装 Humble、colcon 和所需系统依赖 |
| setup_python_envs.sh | 固定依赖，重建 .venv-ros 和隔离的 .venv-train |
| source_env.sh | 当前 Bash shell 加载 Humble 与 ROS 虚拟环境，拒绝混入 Jazzy |
| source_train.sh | 新 Bash shell 加载独立训练环境，拒绝混入 ROS |
| install_device_rules.sh | 安装 CH343P 串口别名和用户组权限；需重新登录/插拔 |
| create_ubuntu22_vm.py | 校验官方 Jammy 镜像并创建独立的 VMware 虚拟机 |
| install_vm_desktop.sh | 安装 Ubuntu 桌面、HWE 内核、VMware 桌面集成及自动登录 |
| install_vm_project.sh | 安装 Humble/固定环境、构建与完整自检；--resume 从构建继续 |
| install_vm_launchers.sh | 安装仿真、自检、终端入口和安全的模拟自动启动 |
| setup_training_windows.ps1 | 重建独立 Python 3.10 CUDA 训练环境；-Verify 执行两个尺寸自检 |
| fetch_ros_install_cache.py / use_rosdep_cache.sh | 下载并校验官方 ROS 源包和 rosdep 数据，用于客体网络故障 |
| install_ros2_jazzy.sh | 旧 Ubuntu 24.04 虚拟机的历史安装脚本 |

[迁移与验收步骤](../../docs/UBUNTU22_MIGRATION.md)。不复制 Windows 虚拟环境。

当前已安装的 VM 和 Windows GPU 入口见 [运行说明](../../docs/VM_HUMBLE.md)。
