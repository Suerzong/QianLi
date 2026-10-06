# qianli_slam

使用官方 slam_toolbox online_async，/scan + /odom + TF，输出 /map 与唯一 map→odom。
不同时运行 AMCL。采用 2D 360° 虚拟 LiDAR，未来 PointCloud2 驱动在感知层替换，不绑定品牌型号。

```bash
ros2 launch qianli_slam slam.launch.py
ros2 run nav2_map_server map_saver_cli -f /tmp/qianli_test_map --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true
```

分辨率 0.05 m，地图更新 1 s，最小移动 0.10 m/转角 0.10 rad。
里程计默认为 commanded odometry，地图匹配可矫正小误差；不宣称实车定位精度。
