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
| qianli_description | URDF / Xacro / Mesh / Joint / TF / RViz 配置 | ✅ 骨架 |
| qianli_bringup | 整机 Launch、参数加载、系统启动 | ✅ 骨架 |
| qianli_interfaces | 自定义 msg / srv / action 集中维护 | ✅ 骨架 |
| qianli_arm | 真实机械臂驱动、关节状态、底层协议 | ⏳ 占位 |
| qianli_manipulation | MoveIt2 / IK / Trajectory / 抓取 / 按按钮 | ⏳ 占位 |
| qianli_base | 移动底盘接口、cmd_vel、轮式里程计 | ⏳ 占位 |
| qianli_control | ros2_control、Controller、底盘/机械臂控制接口 | ⏳ 占位 |
| qianli_localization | IMU / Odometry / EKF / robot_localization | ⏳ 占位 |
| qianli_slam | 2D/3D SLAM、LiDAR Odometry、Map Generation | ⏳ 占位 |
| qianli_navigation | Nav2、Costmap、Planner、Controller | ⏳ 占位 |
| qianli_exploration | Frontier Exploration、Information Gain、自主探索策略 | ⏳ 占位 |
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
    └── base_link
        ├── lidar_link
        ├── camera_link
        ├── imu_link
        └── arm_base_link
            └── ...
                └── end_effector_link
```

原则：

- `map → odom` 由定位 / SLAM 提供；`odom → base_link` 由里程计 / EKF 提供；
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
