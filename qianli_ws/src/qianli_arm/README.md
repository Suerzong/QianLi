# qianli_arm / so101_bringup

完整 SO-ARM101 ROS 驱动包，公开包名保留 **so101_bringup**。包括串口协议、driver_node、ik_node、ik_demo.launch.py、实测原始限位和回归测试。唯一实现保存在 so101_overlay，so101_bringup 命名空间兼容既有导入。

```bash
bash scripts/tools/build.sh
source scripts/setup/source_env.sh
ros2 launch so101_bringup ik_demo.launch.py driver_mode:=sim allow_motion:=false
```

默认启动模型、IK、安全闸门与 RViz；use_rviz:=false 适用于无显示环境。真机流程、资产来源及校验记录见 [迁移说明](../../../docs/UBUNTU22_MIGRATION.md) 和 migration_provenance.json。
