# 标准接口契约（尚无实现）

| 方向 | 接口 | 类型/约定 |
|---|---|---|
| 输入 | /map | nav_msgs/msg/OccupancyGrid，transient_local；-1 未知 |
| 输入 | map→base_footprint | TF2，所有候选目标使用 map frame |
| 查询 | /compute_path_to_pose | nav2_msgs/action/ComputePathToPose；筛除不可达候选 |
| 输出 | /navigate_to_pose | nav2_msgs/action/NavigateToPose；标准 PoseStamped 目标 |
| 状态 | action feedback/result | 距离、恢复次数、成功/失败/取消 |

目标结束或取消后由 Nav2/控制层停止；探索逻辑不成为第二个 cmd_vel 发布者。
未来节点使用 use_sim_time，与地图同一时间源；停止探索时取消当前 action 并等待确认。
