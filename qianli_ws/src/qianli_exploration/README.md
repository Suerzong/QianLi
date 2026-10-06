# qianli_exploration — Frontier Exploration v0.4

从实时 SLAM 地图寻找未知区域边界，自主选择可达观察点，通过 Nav2 持续扩展地图。
使用标准 ROS 接口；探索策略只接收 `/map`、`/scan` 和 TF，不读取场景 manifest、
预建地图或 Gazebo 真值，也不发布 `/cmd_vel`。

## 启动

在 Ubuntu 24.04 / ROS 2 Jazzy 的 QianLi 工作区构建并加载环境：

```bash
source /opt/ros/jazzy/setup.bash
cd qianli_ws
# 新环境先按 package.xml 安装依赖：
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=81 GZ_PARTITION=qianli_exploration_v1
ros2 launch qianli_bringup training.launch.py \
  variant:=train_000 slam:=true nav2:=true explore:=true rviz:=true \
  exploration_report:="$PWD/log/exploration/run.json"
```

现有虚拟机需要已工作的 DISPLAY 或 Xvfb :98 和 `LIBGL_ALWAYS_SOFTWARE=1`，详见
`qianli_sim/README.md`。小场景使用 `qianli_bringup sim.launch.py` 和相同的
`slam:=true nav2:=true explore:=true` 参数。默认 `explore:=false`。
探索启动要求在线 SLAM 和 Nav2；预建地图 AMCL 模式不会被误当作未知区域探索。

若已有一套 SLAM/Nav2 正在运行，可单独启动：

```bash
ros2 launch qianli_exploration exploration.launch.py report_file:=/tmp/exploration.json
```

同一会话由探索节点独占导航目标。不要同时运行手动导航目标、底盘 teleop 或
CEM 局部控制评测。节点等待 Nav2 四个生命周期节点激活，再委托执行约 90° 原地观察，
观察被拒绝或失败时最多重试三次。SLAM 必须支持并启用
`check_min_dist_and_heading_precisely: true`，使纯转动也会接收新扫描。

## 选择与安全边界

1. 在 `/map` 中提取与未知栅格相邻的已知空地，聚类并过滤小碎片。
2. 在已知空地内按整车保守包络计算净空，只选择与机器人连通的区域。
3. 沿边界采样观察点，按附近未知面积与距离评分；目标落在边界的已知一侧。
4. 查询 `/compute_path_to_pose`，检查成功状态、地图 frame，以及每段路径的已知空地净空。
5. 通过 `/navigate_to_pose` 执行，完成后更新地图并选择下一个观察点。

当前包络半径 0.47 m 包含 0.70 m 八边形底盘和现有 0.06 m Nav2 padding 的保守余量；
额外保留栅格半对角线。观察点另留 0.10 m 接近余量（`goal_clearance_margin`），
路径连通性仍按车体包络计算。此圆形包络适用于任意朝向，会放弃一部分整车实际能通过的
紧窄通道。改变整车轮廓或 padding 时，应同步调整 `config/exploration.yaml`。
未知、中间概率和障碍栅格均不可通行；规划器保持 `allow_unknown: false`，
全局 inflation 启用 `inflate_around_unknown: true`，减少沿未知边缘贴边的路径。

## 状态、停止与恢复

- `/exploration/status`：transient-local JSON，含已知栅格增长、目标成功数、失败记录、
  当前状态和目标历史；`report_file` 可将同样内容持续写入磁盘。
- `/exploration/frontiers`：候选观察点 MarkerArray；在 RViz 添加该 topic 可查看。
- `/exploration/enable`：SetBool 服务，关闭时取消当前 action。

```bash
ros2 service call /exploration/enable std_srvs/srv/SetBool '{data: false}'
ros2 topic echo /exploration/status --once
# 确认 phase 为 stopped 后可恢复：
ros2 service call /exploration/enable std_srvs/srv/SetBool '{data: true}'
```

取消确认以前不会发送后继目标；请求尚未被服务器接受时，迟到的接受响应也会被取消。
雷达失效、地图陈旧或 TF 缺失会暂停并取消当前目标；恢复后可继续。
地图分辨率/朝向改变或已知区域显著缩小视为重置，取消旧目标并清空失败缓存。
正常地图扩展、origin 平移和栅格尺寸增长不会被误判为重置。

无法规划或导航失败的目标进入临时黑名单；到达的观察点暂缓重复访问。
目标超时、无运动进展、地图无增长和总时长限制均有处理，默认限制是**墙钟时间**。
`exhausted` 表示连续多轮没有达到最小尺寸的 frontier；它不等于封闭房间、
所有遮挡或全部未知像素都已建图。仍有 frontier 但找不到安全观察点时停止为
`blocked_frontiers`，不会报告完成。`stalled` 和 `duration_limit` 也不算完成。

## 保存地图

探索完成或停止后，SLAM 仍在运行，可保存本次实际扫描地图：

```bash
ros2 run nav2_map_server map_saver_cli -f /tmp/qianli_explored \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -p save_map_timeout:=20.0
```

## 验证

```bash
colcon test --packages-select qianli_exploration --event-handlers console_direct+
colcon test-result --verbose
```

算法测试涵盖未知目标、封闭区域、窄门、连通性、地图 origin/旋转和整段路径检查；
ROS 生命周期测试涵盖迟到接受、取消竞态、传感器失效、规划失败与黑名单。
独立仿真观测器只在评分侧读取真值和场景几何，策略不接收这些信息：

```bash
SHARE=$(ros2 pkg prefix --share qianli_training_scenarios)
ros2 run qianli_exploration exploration_check.py \
  --manifest "$SHARE/generated/train_000/manifest.json" \
  --output /tmp/exploration_check.json --duration 300
```

观测器验证地图增长、实际路程、自动完成目标、采样轮廓碰撞和停止确认。
默认检查是有限时长的探索进展测试；加 `--require-exhausted` 才要求 frontier 耗尽。
记录的已知面积包含已观察到的空地和障碍物，不是整栋楼覆盖率。
现有 `ideal_kinematic_sim` 不能验证真实接触、打滑或实车定位精度。
验收结果见仓库 `docs/EXPLORATION_VALIDATION.md`。
