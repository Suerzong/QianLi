# 自主探索接口

| 方向 | 接口 | 类型/约定 |
|---|---|---|
| 输入 | /map | OccupancyGrid，transient_local，-1 未知 |
| 输入 | /scan | LaserScan，传感器有效性/墙钟超时检查 |
| 输入 | map→base_footprint | TF2，目标与地图使用同一 frame |
| 查询 | /compute_path_to_pose | ComputePathToPose，成功后检查整段路径净空 |
| 输出 | /navigate_to_pose | NavigateToPose，串行委托 Nav2 |
| 输出 | /spin | Spin，仅用于启动时的原地观察 |
| 输出 | /exploration/status | String JSON，transient_local |
| 输出 | /exploration/frontiers | MarkerArray，transient_local |
| 控制 | /exploration/enable | SetBool，关闭请求与停止确认分离 |

探索节点没有 cmd_vel 发布者，也不接收 Gazebo 真值、manifest 或预建地图路径。
每次仅允许一个 Nav2 action 未完成；取消后等待 action result，再选择后继目标。
目标坐标、失败缓存和距离均使用地图坐标，不使用易随地图扩展改变的栅格索引。
仿真节点使用 use_sim_time；传感器、规划、运动进展和取消的等待用墙钟计时。
