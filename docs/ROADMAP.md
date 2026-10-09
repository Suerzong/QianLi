# ROADMAP — 开发路线图

> QianLi 开发路线与 Milestone 定义。当前阶段：Phase 0 — Foundation。

当前 ROS 基线为 **Ubuntu 22.04.5 / ROS 2 Humble / Python 3.10**。Humble VM 与 Windows CUDA 已安装；原生 Ubuntu 及真机运动/抓取仍需验收。下述路线区分历史研发成果与迁移后的验证，详见 [MIGRATION_VALIDATION.md](MIGRATION_VALIDATION.md)。

## Phase 0 — Foundation（当前）

建立可维护、可扩展、可正常 `colcon build` 的 QianLi ROS 2 工作区。

- [x] 项目目录结构与 Git 仓库
- [x] 文档体系（PROJECT / ARCHITECTURE / ROADMAP / ENVIRONMENT / HARDWARE / DEVLOG）
- [x] qianli_ws 工作区骨架 + 基础 package（qianli_interfaces / qianli_description / qianli_bringup）
- [x] setup / build / clean / status 辅助脚本
- [x] 首次构建：旧 Ubuntu 24.04/Jazzy VM（历史记录）
- [x] 当前 Humble VM：6 包构建、23 项 colcon 测试及 36 项迁移回归
- [ ] 原生 Ubuntu 22.04/HWE：Linux GPU、设备及完整软件/硬件验收

## Milestone 1 — 机械臂 ROS2/RViz 建模

- [x] URDF 建模机械臂（**SO-ARM101**，`urdf/so101.urdf`，6 DOF + 夹爪，来自虚拟机 ~/arm-final）
- [x] STL 网格与资产路径整合（`meshes/`，18 个 STL，`package://qianli_description/meshes/`）
- [x] display launch 与 RViz 配置（`launch/display.launch.py`、`rviz/arm.rviz`）
- [x] TF 树与 joint_states 发布验证（Humble VM，模拟及 direct 状态检查）
- [x] RViz 中虚拟机械臂正确显示（Humble VM 桌面）

## Milestone 2 — 真实机械臂接入

- [x] 完整 `so101_bringup` 驱动入库（目录 `qianli_arm`），Humble VM 真实状态反馈及使能拒绝验证
- [ ] 在新环境核对限位内姿态、补采合格标定并验收低速运动、停止与抓取；旧环境已有运动成果见开发日志

## Milestone 3 — MoveIt2

- [ ] end-effector target pose → IK → Motion Planning → 真实机械臂运动（qianli_manipulation）

## Milestone 4 — Camera + AprilTag

- [ ] Camera → AprilTag → TF → 目标在 arm_base_link 下的位置（qianli_perception）
- [ ] MoveIt2 → 机械臂自动移动到目标前

**最终 Demo**：机械臂自动识别带 AprilTag 的"按钮"，移动至按钮前，完成一次按压动作。

## 并行路线 — 虚拟平台（无真实 LiDAR 不阻塞）

以下清单表示本分支尚待集成的能力；[移动仿真分支](https://github.com/Suerzong/QianLi/tree/codex/cloud-model-training) 已有 Jazzy/Gazebo Harmonic 的底盘、SLAM/Nav2、CEM 避障和 Frontier 原型及独立验证，不应记为整个项目尚未开始。跨分支合并与 Humble 适配仍待验收。

- [ ] Gazebo 虚拟移动机器人（四全向轮，qianli_description / qianli_base / qianli_control）
- [ ] 虚拟 LiDAR（Simulation / Mock 模式，qianli_slam）
- [ ] SLAM（先 2D，真实 3D LiDAR 到位后切换 FAST-LIO2 / LIO-SAM）
- [ ] Nav2（Costmap / Planner / Controller，qianli_navigation）
- [ ] Frontier Exploration（qianli_exploration）
- [ ] robot_localization EKF 融合（qianli_localization）

真实 LiDAR（RS-LiDAR-16 / Livox Mid-360）到位后，替换模拟传感器驱动即可。

## 远期路线

YOLO → Semantic Mapping → Natural Language Agent → Multi-floor Navigation → Elevator Interaction

## 里程碑状态总览

| Milestone | 内容 | 状态 |
|---|---|---|
| M0 | 可 colcon build 的工作区 | ✅ Humble VM 六包构建/测试通过；原生部署待验收 |
| M1 | 机械臂 ROS2/RViz 建模 | ✅ Humble VM 模型、joint_states 与 TF 验证通过 |
| M2 | 真实机械臂接入 | 🚧 新 VM 状态读取及使能拒绝通过；标定、运动与抓取待验收 |
| M3 | MoveIt2 | 未开始 |
| M4 | Camera + AprilTag 按压 Demo | 未开始 |
