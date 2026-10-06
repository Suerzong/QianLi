# 地图、车型与训练入口

地图、车型和训练代码统一保存在同一个 QianLi Git 仓库中。此文档是汇总入口；
实际资产保留在各 ROS 包中，避免复制后出现两个版本。构建输出和运行日志写入
`qianli_ws/build`、`install`、`log`，这些目录不提交 Git。

## 文件在哪里

| 内容 | 位置 | 当前用途 |
|---|---|---|
| 四全向轮车型与尺寸 | [qianli_description](../qianli_ws/src/qianli_description/README.md) | 八边形底盘、轮组、Xacro、TF、导航轮廓 |
| 底盘控制 | [qianli_control](../qianli_ws/src/qianli_control/README.md) | ros2_control、Omni X-drive 控制器、运动检查 |
| 雷达、IMU与仿真 | [qianli_sim](../qianli_ws/src/qianli_sim/README.md) | Gazebo Harmonic、/scan、/odom、理想运动执行 |
| 旧教学楼地图 | [qianli_slam/maps](../qianli_ws/src/qianli_slam/maps/README_clean_abstract.md) | 从照片提取的近似地图，尺寸和门洞待实测 |
| 旧教学楼三维场景 | [qianli_sim/worlds](../qianli_ws/src/qianli_sim/worlds/README_clean_abstract.md) | 近似墙体模型；训练优先采用下方同源生成场景 |
| 配套地图、场景、任务 | [qianli_training_scenarios](../qianli_ws/src/qianli_training_scenarios/README.md) | 同一物体列表生成 SDF、PGM、manifest；包含训练/测试划分 |
| 雷达避障参数学习 | [qianli_omni_learning](../qianli_ws/src/qianli_omni_learning/README.md) | CPU CEM 参数搜索、冻结策略、离线及 ROS 评测 |
| SLAM | [qianli_slam](../qianli_ws/src/qianli_slam/README.md) | 在线二维建图 |
| 导航 | [qianli_navigation](../qianli_ws/src/qianli_navigation/README.md) | Nav2、已知地图定位、规划和避障 |
| 自主探索 | [qianli_exploration](../qianli_ws/src/qianli_exploration/README.md) | 已实现 frontier 基线、Nav2 委托和停止/恢复 |
| 统一场景启动 | [training.launch.py](../qianli_ws/src/qianli_bringup/launch/training.launch.py) | 选择场景、在线 SLAM 或预建地图定位、Nav2 |

`simulation/` 是早期规划目录；当前可运行资产以 `qianli_ws/src/` 中的 ROS 包为准。
旧地图的照片推导尺寸与合成教学楼的设计尺寸不可混用。SDF 与 PGM/YAML 必须来自
同一个场景版本，替换训练场景时一起替换。

## 运行已有教学楼

环境：Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。完整依赖和软件渲染设置见
[项目 README](../README.md) 与 [场景说明](../qianli_ws/src/qianli_training_scenarios/README.md)。
以下在仓库根目录运行，使用现有开发虚拟机或安装相同依赖的 Linux 主机。

```bash
source /opt/ros/jazzy/setup.bash
cd qianli_ws
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=78
export GZ_PARTITION=qianli_teaching_training_v1
ros2 launch qianli_bringup training.launch.py variant:=baseline rviz:=true
```

默认加载预建地图并使用 AMCL，适合检查导航。要从未知地图开始在线建图，将最后
一条命令替换为：

```bash
ros2 launch qianli_bringup training.launch.py variant:=train_000 slam:=true rviz:=true
```

`slam:=true` 禁用预建地图 AMCL，由 slam_toolbox 维护 `/map` 和 `map -> odom`。
该命令只启动在线建图。增加 `explore:=true` 后会自动选取 frontier 目标，例如：

```bash
ros2 launch qianli_bringup training.launch.py variant:=train_000 slam:=true nav2:=true explore:=true rviz:=true
```
同一 ROS Domain/Gazebo Partition 中只运行一套仿真，并在所有配套终端加载相同环境。
现有虚拟机的 GPU LiDAR 需要正常的桌面 DISPLAY 或 Xvfb；不能仅因收到导航成功结果
就判断传感器正常，应检查 `/scan` 持续含有有效距离。

## 运行已有参数训练

`qianli_omni_learning` 学习局部避障控制的八个参数，使用 CPU CEM；它不是 PPO
神经网络，也没有学习整栋楼的探索目标。训练仅使用 `train_000/train_001`，
冻结策略后再评测 `test_101`。当前划分主要改变障碍布置，任务指令保持相同。

在已经构建并加载环境的 `qianli_ws` 目录中：

```bash
scene_share="$(ros2 pkg prefix --share qianli_training_scenarios)"
learning_share="$(ros2 pkg prefix --share qianli_omni_learning)"
run_dir="$PWD/log/omni_learning_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
ros2 run qianli_omni_learning train.py \
  --scene-root "$scene_share/generated" \
  --tasks "$learning_share/config/omni_tasks.json" \
  --output "$run_dir/training" \
  --iterations 8 --population 24 --workers 4 --seed 20261005
```

源码中的 `qianli_omni_learning/policies/` 保存冻结参数、训练历史和离线评测记录。
新的实验输出保存在 `log/`，不会覆盖这些冻结记录。ROS/Gazebo 独立任务评测见该包
README；它会禁用 Nav2，防止两个控制器同时向 `/cmd_vel` 发指令。

## 自主探索接在哪里

`qianli_exploration` 已实现 frontier 提取、保守净空/连通性筛选、信息增益评分、
Nav2 路径查询、导航委托、黑名单与取消处理。通过 `explore:=true` 与在线 SLAM
一起启动。状态、停止服务、保存地图命令见
[探索说明](../qianli_ws/src/qianli_exploration/README.md)，验证范围见
[验收记录](EXPLORATION_VALIDATION.md)。

当前为确定性探索基线。下一阶段先验证整栋楼覆盖、回环和真实传感器，再训练
目标选择策略。CEM 局部控制器与 frontier 目标选择属于不同层次，测试分别进行。

## 版本与同步

本次汇总以 ROS 虚拟机提交 `16b9e0015390aa16c457c77315ac0eaff225bbfe` 为基础，
纳入当时未提交的全向底盘学习包、训练说明、旧场景墙体更新和楼层启动入口。
Windows 上机械臂开发分支的未提交改动保留在原工作区。

在其他机器复用本次云端汇总：

```bash
git clone --branch codex/cloud-model-training https://github.com/Suerzong/QianLi.git
```

汇总分支合并到主分支后，后续机器统一从 `main` 获取更新；多台机器各自修改时
使用独立分支和 PR 汇合，避免把不同工作区的完整目录互相覆盖。

## 本次汇总验证（2026-10-06）

- 在虚拟机临时工作区独立 `colcon build`：11 个包构建成功。
- 车型几何测试：2 tests，0 errors，0 failures；全向轮运动学检查通过。
- 统一楼层启动入口 `--show-args` 加载成功。
- 四组训练场景重新生成并通过门洞、连通性和 SDF/map 几何校验；
  manifest、SDF、PGM、YAML 和 validation 与提交版本逐字节一致。
- 冻结策略离线复测：基线与 CEM 策略均 7/7，通过且没有原始/扩展轮廓穿障碍；
  总模拟时间分别 132.1 s 和 119.8 s。冻结策略原始 JSON 字节和 SHA-256 保留。
- 本次没有重新运行完整 Gazebo 14 回合对比，也没有进行真机测试；
  既有 ROS 对比记录与命令见学习包 README。

- 自主探索最终仿真：小场景 6 个目标、19.87 m，frontier 耗尽验收通过；
  教学楼 2 个目标、6.36 m，探索进展验收通过；两次原始/扩展轮廓相交采样均为零。
  构建与 22 个行为回归用例通过，原始记录见 [自主探索验收](EXPLORATION_VALIDATION.md)。
