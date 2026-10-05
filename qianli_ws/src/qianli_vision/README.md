# qianli_vision — RGB 相机物块定位 + 抓取桥接

最新仿真抓取入口为 `scripts/sim_grasp_ok.py`，窗口入口为 `scripts/view_grasp_ok.py`。
碰撞修复、重试策略、验收结果和复现命令见 [仿真抓取验收](../../../docs/GRASP_SIM_VALIDATION.md)。

用普通 RGB 相机 + 3.3cm 网格纸标尺，实现"物块检测 → 物理坐标 → 机械臂抓取"链路。

## 完整链路

```
相机采集 → 灰色物块检测 → 网格纸 Homography 标定 → 物理坐标(cm)
    → grid→base_link 外参变换 → /arm/target_position → ik_node → 机械臂
```

## 节点

| 节点 | 作用 | 用法 |
|---|---|---|
| `object_localizer` | 检测物块，标定网格纸，发布 /object_pose（grid 系） | `ros2 run qianli_vision object_localizer` |
| `grab_bridge` | /object_pose(grid) → 外参变换 → /arm/target_position(base_link) | `ros2 run qianli_vision grab_bridge` |

## 学习脚本（scripts/，教学用）

| 脚本 | 内容 |
|---|---|
| `detect_gray.py` | HSV 灰色识别原理（S 低 + V 适中），5 窗口可视化 |
| `detect_gray_roi.py` | ROI 排除灰色桌面干扰 |
| `detect_gray_trackbar.py` | 滑块实时调参 |
| `detect_gray_multi.py` | 多物块显示 + mask 叠加 |
| `detect_diagnose.py` | 诊断模式：显示所有轮廓+面积落盘 |
| `auto_grid_calib.py` | 自动网格线检测 + Homography 标定 |
| `mask_diagnose.py` | 尺寸过滤（30~50px）掩码诊断 |
| `calibrate_grid.py` | 手动点击格点标定（备选） |

## 标定原理

灰色判定（HSV）：`S < 127` 且 `60 < V < 167`（S 低=颜色淡，V 适中=不黑不白）

网格标定：Hough 检测横竖线 → 行线/列线聚类 → 交点映射
（第 i 行线 = y=i×3.3cm）→ `cv2.findHomography` → H 矩阵

物块尺寸过滤：只保留最长边在 [min_size, max_size] 像素的物块（默认 30~50）。

## 外参标定（grid → base_link，需要用户操作）

用机械臂末端触碰网格纸原点（标定时的 (0,0) 角），记录：
- grid_origin_x/y/z：该角在 base_link 系的位置（米）
- grid_theta_deg：网格纸 x 轴相对 base_link 的角度（度）

然后：
```bash
ros2 run qianli_vision grab_bridge --ros-args \
  -p grid_origin_x:=0.20 -p grid_origin_y:=0.0 \
  -p grid_origin_z:=0.02 -p grid_theta_deg:=0.0
```

## 验证记录

- 标定：行线 6 条、列线 8 条，H 矩阵 RANSAC 计算成功
- 物块定位：X=26.2cm Y=25.6cm（像素 306,291），读数稳定
- 物块尺寸：24×25 px（很小，注意 30px 下限会滤掉它）
