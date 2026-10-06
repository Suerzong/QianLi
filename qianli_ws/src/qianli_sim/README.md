# qianli_sim — ideal_kinematic_sim

Gazebo Harmonic 8，实际 gz_ros2_control/GazeboSimSystem 与官方 omni controller。
四轮 position/velocity 来自 Gazebo joint state；默认里程计来自官方控制器 open_loop commanded odometry。
ideal_kinematic_sim 读取该 /odom 的 body twist，送入 Gazebo 自带 VelocityControl。
Gazebo 的独立 OdometryPublisher 只输出 /simulation/ground_truth，不桥接它的 TF。

当前抽象：机器人重力与接触禁用，模型速度被直接施加；普通 cylinder 不承担全向轮接触动力学。
这验证 ros2_control、上层运动、场景传感器与 SLAM/Nav2 软件链，不能验证摩擦、打滑、力矩、碰撞停止或真实质量。
Nav2 根据激光和 footprint 避障；Gazebo ideal 模型本身可穿障碍，测试必须同时检查轨迹是否穿障碍。
wheel_physics_sim 独立 TODO：优先评估 Harmonic DART 对 anisotropic friction 的实现，再考虑真实滚子接触。
不靠修改运动学来补偿普通圆柱轮横移失败。

惯性集中在 description/urdf/sim_inertials.xacro，明确 TODO/PLACEHOLDER：
“仅供初步仿真，等待实车称重后更新。”默认静态几何入口没有惯性。

```bash
ros2 launch qianli_sim sim.launch.py gui:=true rviz:=true
# 软件 OpenGL / 虚拟显示环境可无 GUI：
ros2 launch qianli_sim sim.launch.py gui:=false rviz:=false
ros2 run qianli_control motion_test.py --gazebo --ros-args -p use_sim_time:=true
```

静态 bridge 配置统一在 config/bridge.yaml。
`/cmd_vel` 为唯一上层 TwistStamped 接口；`/simulation/body_velocity` 是内部理想运动执行接口。
所有仿真 ROS 节点 use_sim_time=true。源模型/world 不依赖在线资源。

虚拟 IMU 安装在 base_link，50 Hz，`/imu/data` / `imu_link`；虚拟 2D LiDAR
距地面 0.25 m，360 束覆盖 360°，10 Hz，量程 0.12–12 m，`/scan` / `lidar_link`。
这些位置仅用于软件原型，后续替换为实测安装位姿；未来 3D PointCloud2 接口不绑定传感器型号。

Gazebo 原始扫描保留在 `/simulation/scan_raw`。仿真适配器只把健康扫描中的
`+inf` 无命中读数转换为 `range_max - 0.001 m`（当前 11.999 m），时间戳、frame、
角度和有限障碍读数保持原值；NaN、负无穷和量程内过近读数保持无效。
`qianli_slam` 的建图截断距离为 11.99 m：Karto 可以清空无命中射线，却不会在
12 m 处画出虚假障碍墙。Nav2 的障碍标记距离为 8 m，无命中端点不进入标记范围。

仅当至少 5%（且至少 3 条）原始射线有有效有限距离时才转换，整帧 infinity 的
渲染故障仍保持无效并被上层暂停。这一语义仅用于当前 Gazebo 驱动；实车雷达接入时
应确认无命中/故障的区别，不能直接套用。转换没有场景几何、地图或真值输入。


VM 软件渲染验收使用 Xvfb（已安装），EGL `--headless-rendering` 在本机导致激光全 infinity，
因此默认关闭该选项。无桌面环境先启动 `Xvfb :98 -screen 0 1500x950x24`，
再设置 `DISPLAY=:98 LIBGL_ALWAYS_SOFTWARE=1`。有桌面时直接使用原有 DISPLAY。

```bash
ros2 run qianli_sim check_system.py --ros-args -p use_sim_time:=true
ros2 run qianli_sim check_imu_motion.py --ros-args -p use_sim_time:=true
# 在仓库根目录，建图后增加 --map：
./scripts/test_base_sim.sh --map
```
IMU 动态测试等待 DDS 发现完成，实际发布旋转命令并检查角速度响应及停止。
