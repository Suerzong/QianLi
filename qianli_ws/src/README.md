# qianli_ws/src — ROS 2 Packages

QianLi 全部 ROS 2 包源码目录。Package 状态约定：

| 状态 | 含义 |
|---|---|
| ✅ 骨架 | 已初始化可编译（package.xml + CMakeLists.txt） |
| ⏳ 占位 | 仅目录 + README，未初始化；对应 Milestone 启动时创建 |

| Package | 职责 | 状态 | 启动时机 |
|---|---|---|---|
| qianli_interfaces | 自定义 msg / srv / action | ✅ 骨架 | 已创建 |
| qianli_description | URDF / Xacro / TF / RViz | ✅ 骨架 | 已创建 |
| qianli_bringup | 整机 Launch | ✅ 骨架 | 已创建 |
| so101_bringup（目录 qianli_arm） | 完整驱动 / IK / 安全 launch | ✅ 实现 | Ubuntu 22.04 迁移 |
| qianli_manipulation | MoveIt2 / IK / 抓取 / 按按钮 | ⏳ 占位 | Milestone 3 |
| qianli_base | 底盘接口 / cmd_vel / 里程计 | ⏳ 占位 | 虚拟底盘 |
| qianli_control | ros2_control / Controller | ⏳ 占位 | 虚拟底盘 |
| qianli_localization | IMU / Odom / EKF | ⏳ 占位 | 虚拟底盘 |
| qianli_slam | 2D/3D SLAM / 建图 | ⏳ 占位 | 虚拟 LiDAR |
| qianli_navigation | Nav2 | ⏳ 占位 | 虚拟底盘 |
| qianli_exploration | Frontier 探索 | ⏳ 占位 | Nav2 之后 |
| qianli_perception | Camera / AprilTag / YOLO / OCR / VLM | ⏳ 占位 | Milestone 4 |
| qianli_semantic_map | 语义实体 + 地图坐标 | ⏳ 占位 | 远期 |
| qianli_task_planner | 自然语言任务 / Agent / BT | ⏳ 占位 | 远期 |
| qianli_safety | 安全 / 急停 | ⏳ 占位 | 与底盘并行 |

> colcon 会跳过无 package.xml 的目录（仅提示），不影响整体构建。

当前共 6 个实际包：qianli_description、qianli_bringup、qianli_interfaces、qianli_teach、qianli_vision、so101_bringup。其余是规划占位目录。
