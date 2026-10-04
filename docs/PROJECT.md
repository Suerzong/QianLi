# PROJECT — 项目定义

> QianLi 项目定义文档。记录项目名称来源、目标能力、系统定位、当前阶段与技术选型。

## 1. 项目名称

**QianLi**（千里），取自"千里之行，始于足下"。

## 2. 项目目标

QianLi 是一个面向**室内复杂环境**的自主探索、语义认知与任务执行移动机器人项目。机器人最终应当具备：

1. 在未知室内环境中**自主探索**，不依赖人工提前建图；
2. 利用 **SLAM** 构建环境几何地图；
3. 自动寻找未探索区域并**持续扩展地图**；
4. 在地图中**自主定位、规划路径、动态避障**；
5. 利用**视觉**识别门、房间、走廊、电梯、自动售货机等语义目标；
6. 将视觉语义与空间坐标关联，建立 **Semantic Map**；
7. 接收**自然语言指令**，例如："去自动售货机帮我拿一瓶水"；
8. 将自然语言转化为**结构化机器人任务**；
9. **自主导航**至任务目标；
10. 使用**机械臂**进行抓取、按按钮等操作；
11. 后期实现**电梯交互与多楼层导航**；
12. **长期保存**已探索的地图、语义与环境知识，使机器人逐渐"认识"所在建筑。

## 3. 系统定位

**Autonomous Mobile Manipulator** + **Semantic Navigation** + **Embodied Agent**

中文描述：面向室内复杂环境的自主探索与语义导航移动机器人。

## 4. 当前阶段

**Phase 0 — Foundation**

当前阶段不追求一次实现最终系统。阶段重点是：搭建可维护、可扩展、可正常 `colcon build` 的 ROS 2 工作区，并为后续 Milestone 做好准备。

硬件现状：

- 已有机械臂，可进行真实机械臂开发；
- 真实 3D LiDAR **暂时缺失**（后续可能接入 RoboSense RS-LiDAR-16、Livox Mid-360 或其他 3D LiDAR）；
- 后续接入四麦克纳姆轮移动底盘；
- 后续可能使用 STM32 作为实时底层控制器；
- 后续使用 RGB / RGB-D Camera、IMU、编码器、下视 ToF 等传感器。

> **雷达相关模块当前必须支持 Simulation / Mock 模式**，不要因为没有真实 LiDAR 阻塞项目开发。

## 5. 技术栈

目标平台：**Ubuntu 22.04 + ROS 2 Humble**

| 类别 | 技术 |
|---|---|
| 主要语言 | C++、Python |
| 机器人中间件 | ROS 2（Topic / Service / Action） |
| 建模与可视化 | URDF、Xacro、TF2、RViz2 |
| 控制 | ros2_control、MoveIt2 |
| 仿真 | Gazebo |
| 导航 | Nav2、robot_localization |
| 未来感知/智能 | FAST-LIO2 / LIO-SAM、Frontier Exploration、YOLO、OpenCV、AprilTag、OCR、Semantic Mapping、VLM、LLM Agent、Behavior Tree、Multi-floor Topological Navigation |
| 版本控制 | Git |

## 6. 核心设计原则

1. **Linux 上位机负责**：SLAM、Navigation、Perception、Semantic Mapping、Task Planning、Agent、MoveIt2；
2. **STM32 等 MCU 负责**：电机实时控制、Encoder、PID、底层传感器、安全保护、急停；
3. **禁止**：让 STM32 承担 SLAM；让 Linux 直接生成电机 PWM；
4. 系统之间通过 **ROS 2 Topic / Service / Action** 以及**串口、CAN、Ethernet** 等接口解耦；
5. **所有真实硬件尽量具有 Simulation / Mock 替代实现**。

## 7. ROS 坐标系规范

未来系统至少使用：

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

严格区分 `map` / `odom` / `base_link` / 传感器 frame / 机械臂 frame，**禁止为了临时跑通程序而随意破坏 TF 结构**。

## 8. 地图架构

系统不是只有一张地图，需要区分：

| 地图 | 内容 |
|---|---|
| Geometry Map | 三维点云 / 几何环境 |
| Navigation Map | 2D Occupancy Grid / Costmap |
| Semantic Map | 房间、门、电梯、自动售货机等语义实体及对应地图位置 |
| Topological Map | 楼层、房间、电梯等高层连接关系 |

示例：

```
Building
├── Floor_1
│   ├── Elevator_A
│   └── Vending_Machine
├── Floor_2
│   └── Elevator_A
└── Floor_3
    ├── Elevator_A
    └── Lab_301
```

## 9. Agent 架构原则

- **LLM 不能直接控制电机**；
- LLM / Agent 只进行**高层任务规划**；
- 底层运动由 **Nav2 / MoveIt2 / Controller** 等确定性模块完成。

示例："帮我拿瓶水" →

```
find(vending_machine)
navigate_to(vending_machine)
detect(water)
pick(water)
navigate_to(user)
```

## 10. 开发路线

1. 优先：ROS2 → URDF/Xacro → TF2 → 机械臂 Joint State → MoveIt2 → Camera → AprilTag → 手眼坐标转换 → 机械臂视觉定位 → 自动按目标按钮；
2. 并行：Gazebo → 虚拟移动机器人 → 虚拟 LiDAR → SLAM → Nav2 → Frontier Exploration；
3. 真实 LiDAR 到位后，替换模拟传感器驱动即可；
4. 之后：YOLO → Semantic Mapping → Natural Language Agent → Multi-floor Navigation → Elevator Interaction。

详见 [ROADMAP.md](ROADMAP.md)。
