# HARDWARE — 硬件清单与规划

> 记录 QianLi 硬件现状、规划型号、接口方案与缺失项对策。

## 1. 现有硬件

| 硬件 | 型号 / 规格 | 接口 | 状态 |
|---|---|---|---|
| 机械臂 | [待补充]（真实机械臂，可开发） | 待确认（串口 / USB / CAN） | ✅ 在役 |
| 开发上位机 | 开发机（Windows 11）→ 目标 Linux 主机 | — | 规划中 |

## 2. 规划硬件

| 硬件 | 规划型号 | 用途 | 状态 |
|---|---|---|---|
| 3D LiDAR | RoboSense RS-LiDAR-16 / Livox Mid-360（或其他） | SLAM / 建图 | ⏳ 缺失，先 Simulation/Mock |
| 移动底盘 | 四麦克纳姆轮 | 全向移动 | ⏳ 待接入 |
| 底层控制器 | STM32 | 电机实时控制 / PID / 安全 | ⏳ 待接入 |
| RGB / RGB-D Camera | [待补充] | 视觉感知（AprilTag / YOLO） | ⏳ 待接入 |
| IMU | [待补充] | 里程计融合（EKF） | ⏳ 待接入 |
| 编码器 | 底盘轮毂编码器 | 轮式里程计 | ⏳ 待接入 |
| 下视 ToF | [待补充] | 悬崖检测（安全） | ⏳ 待接入 |

## 3. 接口与通信规划

| 链路 | 方案 |
|---|---|
| 上位机 ↔ 底盘 MCU（STM32） | 串口 / CAN，帧协议封装在 qianli_base |
| 上位机 ↔ 机械臂 | 串口 / USB / Ethernet，封装在 qianli_arm |
| LiDAR / Camera | Ethernet / USB3，ROS 驱动 |
| 急停 / 安全 | MCU 本地直连，独立于 ROS 系统 |

## 4. 坐标系

按 [ARCHITECTURE.md](ARCHITECTURE.md) §4 的 TF 树执行：

```
map → odom → base_link →（lidar_link / camera_link / imu_link / arm_base_link → ... → end_effector_link）
```

## 5. 缺失项与对策

- **真实 3D LiDAR 缺失**：雷达模块必须支持 Simulation / Mock 模式（Gazebo 虚拟雷达），真实 LiDAR 到位后仅替换驱动，不影响上层逻辑；
- **真实底盘缺失**：Gazebo 虚拟底盘先行，验证 SLAM / Nav2 / Exploration 流程；
- **机械臂参数待确认**：自由度、行程、额定负载、通信协议确认后补充到本表与 URDF。

## 6. 硬件接入原则

- 每个硬件模块必须有 Simulation / Mock 替代实现；
- STM32 只负责实时底层（电机 / 编码器 / PID / 安全），不承担 SLAM；
- 驱动代码放在对应 ROS package 内，不散落各处。
