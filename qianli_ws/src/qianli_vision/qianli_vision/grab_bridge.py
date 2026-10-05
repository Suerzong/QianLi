#!/usr/bin/env python3
"""第4步：抓取桥接节点 —— 物块坐标(grid) → base_link → 机械臂抓取

数据流：
  qianli_vision/object_localizer 发布 /object_pose (grid 系)
      ↓ 本节点订阅
  外参变换：grid → base_link（网格纸原点在 base_link 的位置 + 朝向）
      ↓
  发布 /arm/target_position (base_link 系)
      ↓
  ik_node 解算关节角 → 机械臂移动到物块上方 → 夹爪抓取

外参标定（grid → base_link）：
  网格纸坐标系原点 = 网格纸某角（你标定时点过的 (0,0) 角）
  用机械臂末端去碰这个角，记录 base_link 坐标 (x0, y0, z0) 和朝向 θ
  （θ = 网格纸的 x 轴在 base_link 系里的角度，单位度）

用法：
  ros2 run qianli_vision grab_bridge --ros-args \
    -p grid_origin_x:=0.20 -p grid_origin_y:=0.0 \
    -p grid_origin_z:=0.02 -p grid_theta_deg:=0.0 \
    -p approach_z:=0.10
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class GrabBridge(Node):
    def __init__(self):
        super().__init__('grab_bridge')

        # 外参参数：网格纸原点在 base_link 系的位置（米）和朝向（度）
        self.declare_parameter('grid_origin_x', 0.20)
        self.declare_parameter('grid_origin_y', 0.0)
        self.declare_parameter('grid_origin_z', 0.02)
        self.declare_parameter('grid_theta_deg', 0.0)
        # 抓取高度：末端在物块上方多高（米）
        self.declare_parameter('approach_z', 0.10)
        # 是否发布（false = 只打印，安全预览）
        self.declare_parameter('publish', True)

        self.sub = self.create_subscription(
            PointStamped, '/object_pose', self.on_object_pose, 10)
        self.pub = self.create_publisher(
            PointStamped, '/arm/target_position', 10)
        self.get_logger().info(
            '抓取桥接就绪：/object_pose (grid) → /arm/target_position '
            '(base_link)')

    def on_object_pose(self, msg: PointStamped):
        # 1) grid 系坐标（米）
        gx, gy = msg.point.x, msg.point.y

        # 2) 外参：绕 z 旋转 θ，再平移 (x0, y0)
        theta = math.radians(self.get_parameter('grid_theta_deg').value)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        bx = cos_t * gx - sin_t * gy
        by = sin_t * gx + cos_t * gy
        x0 = self.get_parameter('grid_origin_x').value
        y0 = self.get_parameter('grid_origin_y').value
        z0 = self.get_parameter('grid_origin_z').value
        bx += x0
        by += y0

        # 3) 抓取点：物块正上方 approach_z 处
        bz = z0 + self.get_parameter('approach_z').value

        if self.get_parameter('publish').value:
            out = PointStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = 'base_link'
            out.point.x = bx
            out.point.y = by
            out.point.z = bz
            self.pub.publish(out)
            self.get_logger().info(
                f'物块 grid=({gx*100:.1f},{gy*100:.1f})cm → '
                f'base_link=({bx:.3f},{by:.3f},{bz:.3f})m → 已发布')
        else:
            self.get_logger().info(
                f'[预览] 物块 base_link=({bx:.3f},{by:.3f},{bz:.3f})m '
                '(未发布)')


def main():
    rclpy.init()
    node = GrabBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
