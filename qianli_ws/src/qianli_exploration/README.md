# qianli_exploration

可编译的自主探索准备骨架。没有探索 node、控制器、Agent 或 VLM。
下一阶段 Frontier 选择器读取 OccupancyGrid，在 map frame 选择可达目标，
通过标准 NavigateToPose action 委托 Nav2；不直接发布轮速度或 cmd_vel。
接口见 interfaces.md，实施和验收计划见 TODO.md。传感器品牌不进入探索策略。
