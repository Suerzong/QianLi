# qianli_navigation

最小 Nav2：NavFn planner + DWB holonomic controller + smoother + behaviors + BT navigator + lifecycle manager。
允许 vx/vy/wz；max_vel_y=0.25，min_vel_y=-0.25，vy_samples=9，非差速模型。
显式 enable_stamped_cmd_vel=true，controller/behavior 都直接输出唯一上层 /cmd_vel: TwistStamped。
没有 Twist→TwistStamped 转换节点，也没有重复上层命令 topic。

local/global costmap footprint 启动时从 description 的八边形配置注入，footprint_padding=0.06 m，
包含当前圆柱轮约 45.87 mm 的外探余量。未来实机仍需复核完整外形。
SLAM 或 AMCL 负责 map→odom，omni controller 独占 odom→base_footprint。

```bash
ros2 launch qianli_bringup sim.launch.py slam:=true nav2:=true rviz:=true
ros2 run qianli_navigation navigation_test.py --ros-args -p use_sim_time:=true
```

RViz 选择 Nav2 Goal，在地图空闲区按下鼠标拖动确定朝向。
slam:=false nav2:=true 使用已保存测试地图 + AMCL OmniMotionModel，默认初始位姿 (0,0,0)。
SLAM 与 AMCL 互斥。

ideal_kinematic_sim 中模型可穿障碍，因此自动测试同时记录 Gazebo 真值轨迹并检查几何障碍侵入。
软件导航通过不代表真实碰撞动力学通过。

Nav2 Goal 的 action 确认窗口为 1000 ms（适应 VM 软件渲染调度）；成本地图保持真实八边形，
padding=0.06 m 为轮外探与规划余量。AMCL 保存地图模式使用 OmniMotionModel，
它与 SLAM 互斥；不同时发布 map→odom。运行自动验收 `test_base_sim.sh --map --navigation`。

## 未知区域探索模式

`exploration_mode:=true` 使用 Smac2D + RPP，禁用未知区域通行和路径平滑。
`qianli_bringup` 的 `explore:=true` 会自动启用此配置。
局部地图使用含安全余量的 0.47 m 圆形包络，全局另加栅格半对角线与
0.005 m 插值余量；两层都融合实时 SLAM 并保持未知区域。
探索节点检查完整路径后直接调用 FollowPath，地图变化时检查剩余路径。
RPP 在该模式只使用前进和转向，暂不发挥底盘横移能力。
配置依据与实测限制见仓库 `docs/EXPLORATION_COVERAGE.md`。
