# ROADMAP — 开发路线图

> QianLi 开发路线与 Milestone 定义。当前底盘软件阶段：QianLi Simulation v0.3。

## Phase 0 — Foundation（已建立）

建立可维护、可扩展、可正常 `colcon build` 的 QianLi ROS 2 工作区。

- [x] 项目目录结构与 Git 仓库
- [x] 文档体系（PROJECT / ARCHITECTURE / ROADMAP / ENVIRONMENT / HARDWARE / DEVLOG）
- [x] qianli_ws 工作区骨架 + 基础 package（qianli_interfaces / qianli_description / qianli_bringup）
- [x] setup / build / clean / status 辅助脚本
- [x] 在 Ubuntu 24.04 虚拟机（ROS 2 Jazzy，192.168.26.128）上验证首次 `colcon build`

## Milestone 1 — 机械臂 ROS2/RViz 建模

- [x] URDF 建模机械臂（**SO-ARM101**，`urdf/so101.urdf`，6 DOF + 夹爪，来自虚拟机 ~/arm-final）
- [x] STL 网格与资产路径整合（`meshes/`，18 个 STL，`package://qianli_description/meshes/`）
- [x] display launch 与 RViz 配置（`launch/display.launch.py`、`rviz/arm.rviz`）
- [ ] TF 树与 joint_states 发布验证（VM 构建 + 无头验证）
- [ ] RViz 中虚拟机械臂正确显示（虚拟机桌面）

## Milestone 2 — 真实机械臂接入

- [ ] ROS Joint Command → 机械臂 Driver → 真实机械臂（qianli_arm）
- [ ] 真实机械臂状态反馈至 ROS 2

## Milestone 3 — MoveIt2

- [ ] end-effector target pose → IK → Motion Planning → 真实机械臂运动（qianli_manipulation）

## Milestone 4 — Camera + AprilTag

- [ ] Camera → AprilTag → TF → 目标在 arm_base_link 下的位置（qianli_perception）
- [ ] MoveIt2 → 机械臂自动移动到目标前

**最终 Demo**：机械臂自动识别带 AprilTag 的"按钮"，移动至按钮前，完成一次按压动作。

## 并行路线 — 虚拟平台（无真实 LiDAR 不阻塞）

- [x] Gazebo 虚拟移动机器人（四全向轮 Omni X-drive，qianli_description / qianli_base / qianli_control）
- [x] 虚拟 LiDAR（Simulation / Mock 模式，qianli_slam）
- [x] SLAM（先 2D，真实 3D LiDAR 到位后切换 FAST-LIO2 / LIO-SAM）
- [x] Nav2（Costmap / Planner / Controller，qianli_navigation）
- [x] Frontier Exploration 基线（qianli_exploration；整栋楼覆盖验收待办）
- [ ] robot_localization EKF 融合（qianli_localization）

真实 LiDAR（RS-LiDAR-16 / Livox Mid-360）到位后，替换模拟传感器驱动即可。

## 远期路线

YOLO → Semantic Mapping → Natural Language Agent → Multi-floor Navigation → Elevator Interaction

## 里程碑状态总览

| Milestone | 内容 | 状态 |
|---|---|---|
| M0 | 可 colcon build 的工作区 | ✅ 结构完成 + VM 构建验证通过（Ubuntu 24.04 / Jazzy） |
| M1 | 机械臂 ROS2/RViz 建模 | 未开始 |
| M2 | 真实机械臂接入 | 未开始 |
| M3 | MoveIt2 | 未开始 |
| M4 | Camera + AprilTag 按压 Demo | 未开始 |

## QianLi Simulation v0.3（2026-10-05）

- [x] Base Geometry v0.1 / REP-103 / outward joint axes / Xacro 参数化
- [x] 修正为四 Omni Wheel X-drive / 官方 omni controller / mock 实际输出与四运动测试
- [x] Harmonic + gz_ros2_control + ideal_kinematic_sim / 独立 ground truth
- [x] 虚拟 360° 2D LiDAR / IMU 动态响应 / 静态 ros_gz bridge / 同一 sim_time
- [x] slam_toolbox / 唯一 map→odom / 保存测试地图
- [x] Nav2 holonomic / 三目标 action 成功 / 独立轨迹障碍相交检查
- [x] 模块化 bringup / 自动系统验收
- [x] qianli_exploration 可编译骨架、接口与 TODO
- [x] 实现 Frontier Exploration 与标准 Nav2 action 委托
- [ ] 整栋楼覆盖/回环与多出生点探索验收
- [ ] wheel_physics_sim：Harmonic 各向异性接触/滚子接触独立评估
- [ ] 整车称重、重心/惯量、实机电机/编码器正方向校准
- [ ] STM32 hardware interface、反馈里程计、EKF、安全急停
- [ ] 实际 LiDAR 安装标定、3D PointCloud2 接口接入

当前仿真可检验上层软件链，不能代替真实轮子接触动力学或实机安全验收。
机械臂历史路线不因本次仿真而标记完成。
