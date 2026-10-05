# ARCHITECTURE — 软件架构

> QianLi 软件架构文档：计算职责划分、模块（ROS 2 packages）、通信接口、TF 树、地图与 Agent 架构。

## 1. 计算职责划分

### 1.1 Linux 上位机（主计算单元）

SLAM、Navigation、Perception、Semantic Mapping、Task Planning、Agent、MoveIt2。

### 1.2 STM32 等 MCU（实时底层）

电机实时控制、Encoder、PID、底层传感器、安全保护、急停。

### 1.3 职责边界（禁止项）

- ❌ 让 STM32 承担 SLAM / 导航 / 感知等上层计算；
- ❌ 让 Linux 直接生成电机 PWM；
- ✅ 系统间通过 ROS 2 Topic / Service / Action 与串口 / CAN / Ethernet 等接口解耦；
- ✅ 每个真实硬件模块都有对应的 Simulation / Mock 实现。

## 2. 模块架构（ROS 2 Packages）

工作区位于 `qianli_ws/src`：

| Package | 职责 | 当前状态 |
|---|---|---|
| qianli_description | URDF / Xacro / Mesh / Joint / TF / RViz 配置 | ✅ 几何 v0.1，可选 control/仿真传感器 |
| qianli_bringup | 模块化仿真 Launch、隔离子 launch 参数 | ✅ v0.3 |
| qianli_interfaces | 自定义 msg / srv / action 集中维护 | ✅ 骨架 |
| qianli_arm | 真实机械臂驱动、关节状态、底层协议 | ⏳ 占位 |
| qianli_manipulation | MoveIt2 / IK / Trajectory / 抓取 / 按按钮 | ⏳ 占位 |
| qianli_base | 移动底盘接口、cmd_vel、轮式里程计 | ⏳ 占位 |
| qianli_control | 官方 omni controller、mock/Gazebo、teleop、运动学测试 | ✅ v0.3 底盘 |
| qianli_localization | IMU / Odometry / EKF / robot_localization | ⏳ 占位 |
| qianli_slam | slam_toolbox、2D Map、测试地图 | ✅ v0.3 |
| qianli_navigation | Nav2 holonomic DWB / NavFn / AMCL / 自动导航验收 | ✅ v0.3 |
| qianli_exploration | Frontier 标准接口、TODO | ✅ 可编译骨架，尚无探索算法 |
| qianli_perception | Camera、AprilTag、YOLO、OCR、VLM | ⏳ 占位 |
| qianli_semantic_map | Semantic Entity + Map Coordinate 绑定 | ⏳ 占位 |
| qianli_task_planner | 自然语言任务、Agent、Behavior Tree、任务状态机 | ⏳ 占位 |
| qianli_safety | Cliff Detection、Emergency Stop、Sensor Failure、Safety State | ⏳ 占位 |

> 状态说明：**骨架** = 已初始化可编译的 ROS 2 package（package.xml + CMakeLists.txt）；**占位** = 仅创建目录与 README，待对应 Milestone 启动时初始化为可编译 package。colcon 会跳过无 package.xml 的目录（仅提示），不影响构建。

## 3. 通信架构

- 层间通信：ROS 2 **Topic**（传感器数据流、cmd_vel）、**Service**（查询/配置）、**Action**（长时任务：导航、抓取）；
- 上位机 ↔ MCU：**串口（UART）/ CAN / Ethernet**，帧协议封装在 driver package（qianli_arm / qianli_base）内部；
- 上层模块不感知具体总线细节。

## 4. TF 树

```
map
└── odom
    └── base_footprint
        └── base_link
            ├── lidar_link
            ├── camera_link (future)
            ├── imu_link
            └── arm_base_link (future integration)
                └── ...
                    └── end_effector_link
```

原则：

- `map → odom` 由定位 / SLAM 提供；`odom → base_footprint` 当前由官方 omni controller 唯一发布；未来切换 EKF 时关闭控制器 TF；
- 传感器 frame 从 `base_link` 静态 TF 挂载；
- 机械臂 frame 链由 URDF 定义，`end_effector_link` 由 MoveIt2 / 手眼标定维护。

## 5. 地图架构

| 地图 | 内容 | 负责模块 |
|---|---|---|
| Geometry Map | 三维点云 / 几何环境 | qianli_slam |
| Navigation Map | 2D Occupancy Grid / Costmap | qianli_navigation |
| Semantic Map | 语义实体 + 地图坐标（如 `VendingMachine_01: floor 1, x 12.4, y 7.8`） | qianli_semantic_map |
| Topological Map | 楼层 / 房间 / 电梯高层连接关系 | qianli_task_planner（远期） |

## 6. Agent 架构

LLM / Agent 只做高层任务规划，**不直接控制电机**：

```
用户指令（自然语言）
    ↓  [qianli_task_planner]
结构化任务：find(vending_machine) → navigate_to(...) → detect(water) → pick(water) → navigate_to(user)
    ↓
确定性执行：Nav2（导航） / MoveIt2（操作） / Controller（底层）
```

## 7. 数据流示例（拿水任务）

```
用户: "去自动售货机帮我拿一瓶水"
Agent: find(vending_machine)          # 查询 Semantic Map → 坐标
       navigate_to(vending_machine)   # Nav2 导航
       detect(water)                  # 视觉识别（YOLO / AprilTag）
       pick(water)                    # MoveIt2 规划 + 机械臂执行
       navigate_to(user)              # 回到用户位置
```

## 8. 设计约束

- **Clean / Modular / Documented / Buildable**；
- 每个阶段完成后检查 `git status` 与 `colcon build`，保证工作区始终健康；
- 不为"看起来完整"生成无实际意义的模板代码。

## Simulation v0.3 当前实现

```text
Nav2 / teleop / tests
  └─ /cmd_vel (TwistStamped: vx,vy,wz)
       └─ official omni_base_controller → four velocity command interfaces
            ├─ gz_ros2_control → Gazebo wheel joints → /joint_states
            └─ /odom (commanded odometry) + odom→base_footprint
                 └─ ideal_kinematic_sim → /simulation/body_velocity
                       → bridge → Gazebo VelocityControl (body twist)
Gazebo scene → GPU LiDAR → bridge → /scan → SLAM / Nav2 costmaps
Gazebo IMU → bridge → /imu/data
Gazebo OdometryPublisher → bridge → /simulation/ground_truth (no TF)
Gazebo clock → bridge → /clock → all simulation ROS nodes
```

唯一 TF 职责：slam_toolbox（在线建图）或 AMCL（保存地图定位）发布 map→odom；
omni controller 发布 odom→base_footprint；robot_state_publisher 发布 URDF 子树：

```text
map
└── odom
    └── base_footprint
        └── base_link (z=0.09 m)
            ├── front_left_wheel_link
            ├── front_right_wheel_link
            ├── rear_left_wheel_link
            ├── rear_right_wheel_link
            ├── imu_link (fixed, base_link origin)
            └── lidar_link (fixed, ground z=0.25 m)
```

本机官方 controller 4.40.1 支持等角 omni 布局。θ=45°,135°,225°,315° 按 FL/RL/RR/FR。
ωᵢ=(sinθᵢ·vx−cosθᵢ·vy−R·wz)/r，与 outward +joint axis 及右手定则一致，无符号补偿。
R=hypot(0.294,0.294)，r=0.075；几何 launch 从 common.xacro 读取参数，wheel_direction 仅预留实机标定。
open_loop=true，position_feedback=false，cmd_vel_timeout=0.5 s；100 Hz control manager。
closed-loop encoder odometry / EKF 尚未实现，不能称为测量里程计。

Gazebo 机器人 collision 在理想模式的临时 SDF 中移除，源 URDF 保留准确低面数 collision。
保留 scene collision/visual，使真实 Gazebo 激光可检测墙和 boxes。
Nav2 避障验证另用 SAT 多边形/障碍相交检查独立 ground truth 轨迹，不能用“机器人没撞停”判断避障成功。
Nav2 DWB 的 x/y 范围 ±0.25 m/s、平移合速度≤0.30 m/s、wz≤0.6 rad/s；八边形 + padding=0.06 m。
BT action 默认确认等待改为 1000 ms，适应 VM 软件渲染调度。

未来 STM32 SystemInterface 使用同一四关节 velocity command、position/velocity state 接口，
在 hardware layer 校准 fl/fr/rl/rr_direction；不改几何轴向或 controller 矩阵。
未来 3D LiDAR 的 PointCloud2 由感知驱动发布，独立生成 /scan 或切换定位模块；导航逻辑不绑定型号。
惯性仅使用明确的仿真 PLACEHOLDER（底板 10 kg、每轮 0.5 kg），等待实测；不用于动力学结论。

官方依据：[omni controller](https://control.ros.org/jazzy/doc/ros2_controllers/omni_wheel_drive_controller/doc/userdoc.html)、
[gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)。
