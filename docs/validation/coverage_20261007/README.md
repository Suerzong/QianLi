# 覆盖率实测记录

同一 train_000、出生点 [13.5, -16, 0] 和固定 482,123 栅格分母。
评分用参考地图；探索节点只能使用实时 SLAM 地图、雷达和 TF。
这些都是单次仿真实验，没有跨种子统计；部分改动会改变 SLAM 轨迹，因此差异不能全部归因于单个参数。

| 实验 | 可达自由覆盖率 | 评分器墙钟秒数 | 严格完成检查 | 配置/备注 |
|---|---:|---:|---|---|
| baseline | 32.84% | 902.2 | FAIL: premature_duration_limit | 原版 v0.4，900 秒预算 |
| geodesic_only | 33.72% | 364.4 | FAIL: premature_stopped | 仅改通行距离与黑名单，手动停止 |
| visibility_only | 34.67% | 594.1 | FAIL: premature_stalled | 加入可见信息增益，旧 DWB 配置 |
| aligned_dwb | 19.46% | 303.4 | FAIL: premature_stalled | 局部也加栅格余量的过度保守配置 |
| rpp_navfn | 43.75% | 456.7 | FAIL: premature_stalled | RPP + NavFn，连续路径检查拒绝多条路径 |
| smac_follow | 44.46% | 1110.7 | FAIL: premature_stalled | Smac2D + FollowPath，未加 5 cm 跟踪余量 |
| short_lookahead | 7.68% | 124.2 | FAIL: premature_stopped | 短前瞻与较小转向阈值导致不推进，手动停止 |
| tracking_margin | 29.38% | 792.9 | FAIL: premature_stopped | 全局增加 5 cm 余量，暂停后手动停止，覆盖率回退 |
| improved | 44.46% | 1110.7 | FAIL: premature_stalled | 选用实测配置：Smac2D + FollowPath + RPP，撤回 5 cm 余量 |

`check.json` 原始结果保留 FAIL，不将增长或安全停止改写为全楼验收通过。
探索节点和评分器启动有时间差，评分器墙钟与 explorer 的 wall_elapsed_s 并不相同。
原版 coverage 评分器晚启动，画增长曲线仅使用 time_anchor.json 的约 71.92 秒偏移，
原始 history 未修改；最终覆盖率不受时间偏移影响。

每组保存 explorer 状态/目标历史、覆盖率历史、最终 PGM/YAML 和源码/场景 SHA256。
有 runtime_nav2.yaml 的组额外保存了实际 Nav2 参数。improved/at_900s 是最终配置
约 900 秒的快照与当时保存的地图；保存地图的几秒延迟会造成快照指标与 PGM 重算值略有差异。
installed_code.json 核对最终配置的源文件与安装文件。源码 SHA256 不包含 README 等说明文件。

初始 11 包构建见 build.txt；中途三包重建见 rebuild.txt，最终导航配置重建见
final_navigation_build.txt。tests.txt 保留最终 41 个行为用例的输出（colcon 汇总
10 条结果记录，来自 5 个测试入口及其 wrapper；不是只有 10 个行为用例）。
所有实验的几何采样均未记录原始或 padded 底盘碰撞。这是理想运动学的采样结果，
不验证轮胎接触、机械臂外探、打滑、实车定位或扫描间发生的瞬时碰撞。

失败分析：只改目标评分不能修复控制器贴边；为局部车体重复加入栅格余量会堵住门口；
NavFn 返回的部分路径未通过连续车体检查；用 Smac2D 并直接执行已检查路径后能继续推进，
但转弯偏离仍可能使实时地图净空低于车体半径。增加 5 cm 全局跟踪余量后覆盖率回退，已撤回。最终选择 smac_follow 原始实测版本，improved 中的原始测试结果与地图文件为该记录的副本，selection.json 标明选择依据；没有声称另一次完整重跑或多次均值。
缩短 RPP 前瞻距离并降低转向阈值的探测引发不推进，已撤回；该探测的 intervention.json 记录手动停止。

最终结果与按房间统计见[报告](../../EXPLORATION_COVERAGE.md)和 map_metrics.json。
