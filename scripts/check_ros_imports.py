
from project_paths import project_path
#!/usr/bin/env python3
import rclpy
import tf2_ros
import sensor_msgs
import sys
sys.path.insert(0, project_path('qianli_ws/src/qianli_vision/scripts'))
from gripper_model import GripperModel, JOINTS
print('imports OK')
