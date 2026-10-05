# qianli_control — Omni X-drive

使用已安装的官方 `omni_wheel_drive_controller/OmniWheelDriveController`，没有重写控制器。
wheel_names 逆时针 FL→RL→RR→FR，wheel_offset=π/4，robot_radius=hypot(0.294,0.294)=0.415778787 m，wheel_radius=0.075 m。
启动时从 qianli_description/common.xacro 推导几何，避免 launch 参数漂移。

joint +axis 均向外，+velocity 遵循右手定则。
对角位置 θ，轮接地切向 t=(sinθ,-cosθ)，故
`omega=(sinθ*vx-cosθ*vy-R*wz)/r`，与官方 4.40.1 实现一致。
正 wz 时四轮负转；这不是额外符号补偿。

接口：velocity command，position/velocity state；`fl_direction/fr_direction/rl_direction/rr_direction`
均预留为 +1。未来 STM32 hardware layer 负责 motor_direction_multiplier 校准，仿真中不使用符号修补。
本包不实现 STM32 驱动。后续 C++ HardwareInterface 归属此包，可新增 include/ 和 src/。

统一 `/cmd_vel: TwistStamped` → omni controller → 四轮 velocity interfaces。
`/odom: Odometry` 和唯一 `odom→base_footprint` TF 均由 omni controller 发布。
默认 open_loop=true，是 **commanded odometry**，不是实测。
open_loop=false 可使用轮反馈；Mock 的回显也不能当作实车 measured odometry。

```bash
ros2 launch qianli_control control.launch.py
ros2 control list_controllers
ros2 run qianli_control test_xdrive_kinematics.py --live
ros2 run qianli_control motion_test.py
ros2 run qianli_control teleop.py
```

以上为 Mock 无物理模式，用于先验证官方控制器输出。不要与 Gazebo 同时启动 manager。
仿真由 gz_ros2_control 提供 manager，使用同一份 controllers.yaml。
命令超时 0.5 秒，motion test/teleop 退出发送零速度。
仿真终端使用同一个 ROS_DOMAIN_ID、use_sim_time=true；Mock 默认为系统时间。

官方语义依据：
[Jazzy omni controller](https://control.ros.org/jazzy/doc/ros2_controllers/omni_wheel_drive_controller/doc/userdoc.html)，
[4.40.1 wheel command implementation](https://github.com/ros-controls/ros2_controllers/blob/4.40.1/omni_wheel_drive_controller/src/omni_wheel_drive_controller.cpp)。
