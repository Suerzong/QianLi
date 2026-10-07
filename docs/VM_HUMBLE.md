# 已安装的 QianLi Ubuntu 22.04 / Humble 虚拟机

这台 VM 独立安装在 `D:\VMs\QianLi-Ubuntu22-Humble`，不是临时 chroot。旧 `D:\Ubuntu-VM` 和另外两台已有 VM 均保留。

## 打开与运行

Windows 桌面双击 **QianLi - Ubuntu22 Humble**，即可打开新 VM。也可在 VMware「文件 → 打开」选择：

```text
D:\VMs\QianLi-Ubuntu22-Humble\qianli-humble.vmx
```

Ubuntu 自动登录用户 `ros`。桌面入口包括 **QianLi 仿真**、**QianLi 完整自检**、**QianLi 训练自检**、**QianLi 相机**、**QianLi 只读硬件检查**和 **QianLi 终端**。短检查结束后保留结果窗口，按回车关闭。仿真启动时关闭真机运动；自动启动功能也只启动模拟驱动。项目位于 `/home/ros/QianLi`，专用 ROS domain 为 42。已实际验证冷启动、HWE 6.8 内核、自动登录和 RViz 模型显示。

SSH 已配置独立别名：

```powershell
ssh qianli-humble
```

本地桌面密码记录于 VM 目录的 `bootstrap/credentials.txt`，SSH 只接受现有密钥。无需使用该密码运行常规项目命令。

## 仿真、训练与自检

终端入口也可直接执行：

```bash
cd ~/QianLi
bash scripts/tools/qianli.sh sim
bash scripts/tools/qianli.sh check
bash scripts/tools/qianli.sh training-smoke
bash scripts/tools/qianli.sh train 0.04 --steps 10000
bash scripts/tools/qianli.sh camera
bash scripts/tools/qianli.sh devices
```

`check` 包含 Humble/视觉/模拟 ROS 验收、完整迁移回归、20mm 和 40mm 两进程训练短跑与 EGL 渲染。结果保存在 `migration_assets/vm-acceptance/`。ROS 和训练使用各自的 Python 3.10 环境；训练入口会清除继承的 ROS Python/动态库路径，再加载独立环境。

该 VM 的显卡是 VMware 虚拟显卡。GPU 训练使用已安装的 Windows `.venv-train-win`：Windows 桌面双击 **QianLi - GPU Training** 开始当前 40mm 场景训练，输出保存到宿主机项目的 `dual_twin/rl_out/`。修改步数或场景可执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/tools/train_gpu_windows.ps1 -ObjectSize 0.02 -Steps 10000
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/tools/train_gpu_windows.ps1 -ObjectSize 0.04 -Smoke
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/tools/train_gpu_windows.ps1 -ObjectSize 0.04 -Steps 10000 -Pretrain dual_twin/rl_out/bc_policy.zip
```

Windows GPU 与 VM CPU 训练均使用 Torch 2.8.0/cu128 和同一套固定算法依赖。物块尺寸与场景分别记录，历史目录没有元数据时不会被覆盖。

宿主机 RTX 5070 Ti、驱动 591.74、PyTorch CUDA 12.8 已通过实际运算。两个尺寸各完成 1024 步 CUDA 短跑；`bc_policy.zip`、`ppo_bc_final.zip` 都已在新 Python/NumPy 环境加载、预测并验证保存前后权重与输出一致。兼容加载重建归档描述中的 Box 空间并丢弃旧进程的临时回合状态，原模型文件保持不变。40mm 完整训练入口也已完成 BC 权重热启动、checkpoint 和最终模型保存；这只证明运行链路，不代表抓取成功率提升。

## 代码和机器资产

VM 的 Git 分支是 `codex/ubuntu22-humble`。先保存迁移前工作区，再提交迁移实现；机器标定、虚拟环境及安装日志不进入 Git。保全快照位于项目 `.migration-backups/20261007-172250/`，恢复资产在 `migration_assets/recovered/`，持久标定在 `calib/`。

GitHub 在 VM 中发生 TLS 断开时，安装使用宿主机下载的官方 `ros2-apt-source` 包和 rosdep 数据。包通过发布方 SHA256 验证，rosdep 数据绑定具体官方 rosdistro commit，文件来源与校验值见 `migration_assets/ros-install-cache/manifest.json`；不是手工编造依赖映射。ROS shell 自动使用对应本地索引。

## 硬件边界

新 VM 已配置自动连接 CH343 机械臂适配器（`1a86:55d3`）和外置相机（`05a3:9230`）。串口别名为 `/dev/qianli_arm`；相机通过 `/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0` 访问。持久配置在 `~/.config/qianli/environment.sh`，其中 `QI_CAMERA_FOURCC=MJPG` 只配置这台机器；其他安装没有设置时保留 OpenCV 原有格式。相机使用 xHCI 控制器，已验证 640×480 连续 60 帧和实际画面；旧 EHCI 组合出现丢帧及 JPEG 损坏，已修复。

`devices` 检查相机和串口权限，并只读取六个舵机的位置与扭矩状态。用户确认机械臂独立电源尚未打开，因此舵机读取暂未通过；上电后可通过此入口继续检查。该操作不会调用 direct 初始化、切换扭矩、发送目标位置或写 EEPROM。

仿真和自检入口不会自动启动 direct 驱动、开启舵机运动或写 EEPROM。真机必须按 [迁移验收步骤](UBUNTU22_MIGRATION.md) 验证串口、相机、限位和合格标定后再操作。缺失的外参不能用仿真数据或旧错误拟合替代。

安装过程与检查输出在 `migration_assets/vm-install/`。实际验收结果见 [迁移验证记录](MIGRATION_VALIDATION.md)。
