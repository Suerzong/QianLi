#!/usr/bin/env python3
"""SO-ARM101 follower arm inverse-kinematics preview node.

Loads the official so101.urdf with ikpy, solves numeric IK for an end-effector
target and publishes /joint_states so RViz shows the arm following the target.

Target sources (all merged into the same pipeline):
- RViz InteractiveMarker (6-DOF handle at the gripper), provided by this node
- /ik_target (geometry_msgs/PoseStamped) subscribers, e.g. demo trajectories
- optional built-in circle demo: ros2 run ... --ros-args -p demo_circle:=true

The gripper joint is not part of the pose IK and is held at 0.
"""

import json
import math
import tempfile
import time
import warnings
import xml.etree.ElementTree as ET

import numpy as np

warnings.filterwarnings('ignore', message='.*set as active in the active_links_mask.*')

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from rcl_interfaces.srv import GetParameters
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

from interactive_markers import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

from ikpy.chain import Chain

JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]
GRIPPER_JOINT = 'gripper'
IK_JOINT_NAMES = [n for n in JOINT_NAMES if n != GRIPPER_JOINT]

# Geometry-derived operational zero: upper arm points +Z and forearm +X.
# The regular L shape retains Cartesian IK authority unlike a fully extended
# singular pose.
DEFAULT_HOME_POSITIONS = (
    0.0,           # shoulder_pan
    -0.243783220,  # shoulder_lift
    0.282311217,   # elbow_flex
    -0.088104062,  # wrist_flex
    0.0,           # wrist_roll
    0.0,           # gripper
)


def quat_to_matrix(x, y, z, w):
    """Unit quaternion (x, y, z, w) -> 4x4 homogeneous matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), 0.0],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), 0.0],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def matrix_to_quat(m):
    """4x4 rotation matrix -> (x, y, z, w) via Shepperd's method."""
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return x / n, y / n, z / n, w / n


def pose_to_matrix(pose):
    q = pose.orientation
    p = pose.position
    m = quat_to_matrix(q.x, q.y, q.z, q.w)
    m[0, 3], m[1, 3], m[2, 3] = p.x, p.y, p.z
    return m


class So101IkNode(Node):

    def __init__(self):
        super().__init__('so101_ik_node')

        self.declare_parameter('urdf_file', '')
        self.declare_parameter('base_frame', 'base_link')
        # XYZ dragging uses position-only IK.  Wrist roll is removed from the
        # active IK mask in this mode so it keeps the measured hardware angle.
        self.declare_parameter('orientation_mode', 'none')  # none / Z / all
        self.declare_parameter('sync_from_joint_states', True)
        self.declare_parameter('show_rotation_controls', False)
        self.declare_parameter('straight_pose', False)
        self.declare_parameter('max_joint_speed', 3.0)  # rad/s
        self.declare_parameter('control_rate', 50.0)  # Hz
        self.declare_parameter('reach_warn_m', 0.03)
        self.declare_parameter('max_ik_residual_m', 0.003)
        self.declare_parameter('demo_circle', False)
        self.declare_parameter('circle_center', [0.19, 0.0, 0.12])
        self.declare_parameter('circle_radius', 0.04)
        self.declare_parameter('circle_omega', 0.8)  # rad/s
        self.declare_parameter('command_topic', '/joint_commands')
        self.declare_parameter('gripper_open_position', 1.2)
        self.declare_parameter('gripper_closed_position', 0.0)
        self.declare_parameter(
            'home_joint_positions', list(DEFAULT_HOME_POSITIONS))

        base_frame = self.get_parameter('base_frame').value
        self.base_frame = base_frame
        self.orientation_mode = self.get_parameter('orientation_mode').value
        if self.orientation_mode not in ('Z', 'all', 'none'):
            raise ValueError("orientation_mode must be 'Z', 'all' or 'none'")
        self.max_speed = float(self.get_parameter('max_joint_speed').value)
        self.rate = float(self.get_parameter('control_rate').value)
        self.reach_warn = float(self.get_parameter('reach_warn_m').value)
        self.max_ik_residual = float(
            self.get_parameter('max_ik_residual_m').value)
        self.sync_from_joint_states = bool(
            self.get_parameter('sync_from_joint_states').value)
        self.joint_state_synced = not self.sync_from_joint_states
        self.have_driver_status = False
        self._hardware_limits_ready = False
        self._limits_pending = False
        self._limits_client = self.create_client(GetParameters, '/so101_driver/get_parameters')
        self._limits_timer = self.create_timer(1., self._request_hardware_limits)
        # Joints whose IK solution the envelope had to move, and the largest
        # overshoot seen since the last report.  Clipping a solution is normal
        # while a solver walks into a bound, but it must never be invisible.
        self._clip_worst = {}
        self._clip_reported = {}
        self._limits_source = {}
        self._limits_signature = None
        self.motion_enabled = False

        urdf_path = self._resolve_urdf()
        self.chain = Chain.from_urdf_file(
            self._prepare_ik_urdf(urdf_path),
            base_elements=['base_link'],
            name='so101',
        )
        self._apply_active_mask()
        self._urdf_joint_bounds = {link.name:link.bounds for link in self.chain.links if link.name in IK_JOINT_NAMES}

        home_values = [float(value) for value in
                       self.get_parameter('home_joint_positions').value]
        if len(home_values) != len(JOINT_NAMES) or not all(
                math.isfinite(value) for value in home_values):
            raise ValueError(
                'home_joint_positions must contain six finite values')
        self.home_positions = dict(zip(JOINT_NAMES, home_values))

        self.get_logger().info(
            f'IK chain: {len(self.chain.links)} elements, active joints: '
            f'{[self.chain.links[i].name for i, m in enumerate(self.chain.active_links_mask) if m]}')

        n = len(self.chain.links)
        self.q_current = self._ready_pose(n)
        self.q_measured = self.q_current.copy()
        self.q_target = self.q_current.copy()
        self.target_mat = self.chain.forward_kinematics(
            self.q_current, full_kinematics=True)[-1].copy()
        self.target_dirty = False

        command_topic = self.get_parameter('command_topic').value
        self.pub_command = self.create_publisher(
            JointState, command_topic, 10)
        self.sub_target = self.create_subscription(
            PoseStamped, '/ik_target', self._on_target, 10)
        self.sub_position = self.create_subscription(
            PointStamped, '/arm/target_position', self._on_target_position, 10)
        self.sub_gripper = self.create_subscription(
            Float64, '/gripper_command', self._on_gripper_command, 10)
        self.sub_joint_state = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        self.sub_status = self.create_subscription(
            String, '/arm/status', self._on_driver_status, 10)
        self.gripper_target = float(
            self.get_parameter('gripper_closed_position').value)
        self.stop_client = self.create_client(Trigger, '/arm/stop')
        self.enable_client = self.create_client(SetBool, '/arm/enable')
        self.home_service = self.create_service(
            Trigger, '/arm/home', self._on_home)

        self.marker_server = InteractiveMarkerServer(self, 'ik_marker')
        self.menu_handler = MenuHandler()
        self.menu_handler.insert(
            'Gripper: open', callback=self._menu_gripper_open)
        self.menu_handler.insert(
            'Gripper: close', callback=self._menu_gripper_close)
        self.menu_handler.insert(
            'Motion: enable', callback=self._menu_enable_motion)
        self.menu_handler.insert(
            'Motion: disable', callback=self._menu_disable_motion)
        self.menu_handler.insert(
            'Motion: orthogonal home', callback=self._menu_home)
        self.menu_handler.insert(
            'EMERGENCY STOP', callback=self._menu_emergency_stop)
        self._make_marker(base_frame)

        self.pub_axes = self.create_publisher(
            MarkerArray, '/arm/workspace_axes', 1)
        self.axes_timer = self.create_timer(1.0, self._publish_workspace_axes)

        self.last_warn = 0.0
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        if self.get_parameter('demo_circle').value:
            self.declare_parameter('demo_rate', 20.0)
            self._circle_t = 0.0
            demo_rate = float(self.get_parameter('demo_rate').value)
            self.circle_timer = self.create_timer(1.0 / demo_rate, self._circle_tick)
            self.get_logger().info('Circle demo mode enabled on /ik_target')

        self.get_logger().info(
            f'so101_ik_node ready. Drag the pose handle in RViz, or publish '
            f'PointStamped to /arm/target_position (frame: {base_frame}). '
            'Axes: +X front, +Y left, +Z up.')

    def _resolve_urdf(self):
        from pathlib import Path
        p = self.get_parameter('urdf_file').value
        if p:
            return Path(p)
        import ament_index_python.packages as aip
        share = Path(aip.get_package_share_directory('so101_bringup'))
        return share / 'urdf' / 'so101.urdf'

    def _prepare_ik_urdf(self, urdf_path):
        """Build a fork-free URDF copy for ikpy.

        The official so101.urdf branches at gripper_link (moving jaw + tool
        frame). ikpy only supports linear chains, so both branches are cut and
        a tool_tip dummy link is appended at the gripper-frame pose instead.
        """
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        frame_joint = None
        for j in root.findall('joint'):
            if j.get('name') == 'gripper_frame_joint':
                frame_joint = j
        tip_xyz, tip_rpy = '0 0 0', '0 0 0'
        if frame_joint is not None:
            origin = frame_joint.find('origin')
            if origin is not None:
                tip_xyz = origin.get('xyz', tip_xyz)
                tip_rpy = origin.get('rpy', tip_rpy)

        drop_joints = {'gripper', 'gripper_frame_joint'}
        drop_links = {'moving_jaw_so101_v1_link', 'gripper_frame_link'}
        for j in list(root.findall('joint')):
            if j.get('name') in drop_joints:
                root.remove(j)
        for tr in list(root.findall('transmission')):
            jt = tr.find('joint')
            if jt is not None and jt.get('name') == 'gripper':
                root.remove(tr)
        for l in list(root.findall('link')):
            if l.get('name') in drop_links:
                root.remove(l)

        tip_link = ET.SubElement(root, 'link')
        tip_link.set('name', 'tool_tip_link')
        tip_joint = ET.SubElement(root, 'joint')
        tip_joint.set('name', 'tool_tip_joint')
        tip_joint.set('type', 'fixed')
        origin = ET.SubElement(tip_joint, 'origin')
        origin.set('xyz', tip_xyz)
        origin.set('rpy', tip_rpy)
        parent = ET.SubElement(tip_joint, 'parent')
        parent.set('link', 'gripper_link')
        child = ET.SubElement(tip_joint, 'child')
        child.set('link', 'tool_tip_link')

        fd = tempfile.NamedTemporaryFile(
            'wb', suffix='.urdf', delete=False)
        tree.write(fd, encoding='utf-8', xml_declaration=True)
        fd.close()
        return fd.name

    def _apply_active_mask(self):
        # With XYZ-only control, wrist roll is redundant and must retain the
        # physical angle captured from /joint_states instead of drifting.
        active_names = set(IK_JOINT_NAMES)
        if self.orientation_mode == 'none':
            active_names.discard('wrist_roll')
        self.chain.active_links_mask = [
            getattr(link, 'name', None) in active_names
            for link in self.chain.links
        ]

    def _on_driver_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.motion_enabled = bool(payload.get('enabled', False))
            self.have_driver_status = True
        except (TypeError, ValueError, json.JSONDecodeError):
            self._warn_throttled('Rejected malformed /arm/status payload')

    def _on_joint_state(self, msg: JointState):
        if len(msg.name) != len(msg.position):
            return
        values = dict(zip(msg.name, msg.position))
        if any(name not in values or not math.isfinite(values[name])
               for name in JOINT_NAMES):
            return

        link_names = [getattr(link, 'name', None) for link in self.chain.links]
        measured = self.q_measured.copy()
        for name in IK_JOINT_NAMES:
            measured[link_names.index(name)] = float(values[name])
        self.q_measured = measured

        # On startup, and whenever hardware motion is disabled, anchor the IK
        # target and RViz handle to the real arm.  Re-enabling therefore holds
        # the current pose instead of jumping to an old or synthetic target.
        anchor_to_hardware = (
            not self.joint_state_synced or
            (self.have_driver_status and not self.motion_enabled)
        )
        if not anchor_to_hardware:
            return
        # q_current is the outbound, rate-limited command trajectory.  Anchor
        # it to encoder feedback only while motion is disabled/startup;
        # overwriting it during enabled motion erases trajectory progress.
        self.q_current = measured.copy()
        self.q_target = measured.copy()
        self.gripper_target = float(values[GRIPPER_JOINT])
        self.target_mat = self.chain.forward_kinematics(
            measured, full_kinematics=True)[-1].copy()
        self.target_dirty = False
        first_sync = not self.joint_state_synced
        self.joint_state_synced = True
        self._set_marker_pose_from_target()
        if first_sync:
            self.get_logger().info(
                'IK target synchronized from physical /joint_states')

    def _set_marker_pose_from_target(self):
        pose = Pose()
        pose.position.x = float(self.target_mat[0, 3])
        pose.position.y = float(self.target_mat[1, 3])
        pose.position.z = float(self.target_mat[2, 3])
        qx, qy, qz, qw = matrix_to_quat(self.target_mat)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        self.marker_server.setPose('ik_target', pose)
        self.marker_server.applyChanges()

    def _target_input_ready(self):
        if not self.joint_state_synced:
            self._warn_throttled('Waiting for physical /joint_states sync')
            return False
        if self.have_driver_status and not self.motion_enabled:
            self._warn_throttled('Hardware motion is disabled; target ignored')
            return False
        return True

    def _set_cartesian_target(self, candidate):
        if self.orientation_mode == 'none':
            changed = np.linalg.norm(
                candidate[:3, 3] - self.target_mat[:3, 3]) > 1e-6
        else:
            changed = np.linalg.norm(candidate - self.target_mat) > 1e-6
        if not changed:
            return
        self.target_mat = candidate
        self.target_dirty = True

    def _ready_pose(self, n):
        """Bent-forward start pose so the handle starts in the workspace."""
        q = self._home_pose(n)
        if self.get_parameter('straight_pose').value:
            return q
        q.fill(0.0)
        for i, link in enumerate(self.chain.links):
            name = getattr(link, 'name', None)
            if name == 'shoulder_lift':
                q[i] = -1.0
            elif name == 'elbow_flex':
                q[i] = 1.5
            elif name == 'wrist_flex':
                q[i] = -0.7
        return q

    def _home_pose(self, n=None):
        """Return the geometry-derived orthogonal operational-zero pose."""
        if n is None:
            n = len(self.chain.links)
        q = np.zeros(n)
        for index, link in enumerate(self.chain.links):
            name = getattr(link, 'name', None)
            if name in IK_JOINT_NAMES:
                q[index] = self.home_positions[name]
        return q

    def _make_marker(self, frame_id):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = 'ik_target'
        int_marker.description = 'SO-101 end-effector target'
        int_marker.scale = 0.12

        p = self.target_mat[:3, 3]
        qx, qy, qz, qw = matrix_to_quat(self.target_mat)
        int_marker.pose.position.x = float(p[0])
        int_marker.pose.position.y = float(p[1])
        int_marker.pose.position.z = float(p[2])
        int_marker.pose.orientation.x = qx
        int_marker.pose.orientation.y = qy
        int_marker.pose.orientation.z = qz
        int_marker.pose.orientation.w = qw

        ball = Marker()
        ball.type = Marker.SPHERE
        ball.scale.x = 0.025
        ball.scale.y = 0.025
        ball.scale.z = 0.025
        ball.color.r = 0.9
        ball.color.g = 0.3
        ball.color.b = 0.1
        ball.color.a = 0.9

        center = InteractiveMarkerControl()
        center.always_visible = True
        center.interaction_mode = InteractiveMarkerControl.MENU
        center.markers.append(ball)
        int_marker.controls.append(center)

        def axis_control(axis_xyz, mode):
            c = InteractiveMarkerControl()
            c.name = f'{mode}_{axis_xyz}'
            c.orientation.w = 1.0
            c.orientation.x = float(axis_xyz[0])
            c.orientation.y = float(axis_xyz[1])
            c.orientation.z = float(axis_xyz[2])
            c.interaction_mode = mode
            int_marker.controls.append(c)

        for xyz in ('x', 'y', 'z'):
            axis = {'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}[xyz]
            if self.get_parameter('show_rotation_controls').value:
                axis_control(axis, InteractiveMarkerControl.ROTATE_AXIS)
            axis_control(axis, InteractiveMarkerControl.MOVE_AXIS)

        self.marker_server.insert(int_marker)
        self.marker_server.setCallback(int_marker.name, self._on_marker_feedback)
        self.menu_handler.apply(self.marker_server, int_marker.name)
        self.marker_server.applyChanges()

    def _on_marker_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        if not self._target_input_ready():
            return
        if feedback.header.frame_id:
            self._set_cartesian_target(pose_to_matrix(feedback.pose))

    def _on_target(self, msg):
        if not self._target_input_ready() or not self._frame_is_supported(msg.header.frame_id):
            return
        p,q = msg.pose.position,msg.pose.orientation
        values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[3:]) < 1e-12:
            self._warn_throttled('Rejected nonfinite pose or zero quaternion')
            return
        self._set_cartesian_target(pose_to_matrix(msg.pose))

    def _on_target_position(self, msg: PointStamped):
        if not self._target_input_ready():
            return
        if not self._frame_is_supported(msg.header.frame_id):
            return
        if not all(math.isfinite(v) for v in
                   (msg.point.x, msg.point.y, msg.point.z)):
            self._warn_throttled('Rejected non-finite Cartesian target')
            return
        candidate = self.target_mat.copy()
        candidate[0, 3] = msg.point.x
        candidate[1, 3] = msg.point.y
        candidate[2, 3] = msg.point.z
        self._set_cartesian_target(candidate)

    def _frame_is_supported(self, frame_id):
        if frame_id in ('', self.base_frame):
            return True
        self._warn_throttled(
            f'Rejected target in frame {frame_id!r}; expected {self.base_frame!r}')
        return False

    def _publish_workspace_axes(self):
        """Publish the contest coordinate convention beside the arm base."""
        result = MarkerArray()
        axes = (
            ('X', (1.0, 0.0, 0.0), (1.0, 0.15, 0.15), '前', '后'),
            ('Y', (0.0, 1.0, 0.0), (0.15, 1.0, 0.15), '左', '右'),
            ('Z', (0.0, 0.0, 1.0), (0.15, 0.45, 1.0), '上', '下'),
        )
        marker_id = 0
        length = 0.18
        for axis_name, vector, color, positive_name, negative_name in axes:
            for sign, direction_name in ((1.0, positive_name),
                                         (-1.0, negative_name)):
                arrow = Marker()
                arrow.header.frame_id = self.base_frame
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = 'workspace_axes'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.points = [
                    Point(x=0.0, y=0.0, z=0.0),
                    Point(
                        x=sign * length * vector[0],
                        y=sign * length * vector[1],
                        z=sign * length * vector[2],
                    ),
                ]
                arrow.scale.x = 0.005
                arrow.scale.y = 0.014
                arrow.scale.z = 0.02
                arrow.color.r, arrow.color.g, arrow.color.b = color
                arrow.color.a = 0.75
                result.markers.append(arrow)

                label = Marker()
                label.header = arrow.header
                label.ns = 'workspace_axis_labels'
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = sign * (length + 0.025) * vector[0]
                label.pose.position.y = sign * (length + 0.025) * vector[1]
                label.pose.position.z = sign * (length + 0.025) * vector[2]
                label.pose.orientation.w = 1.0
                label.scale.z = 0.022
                label.color.r, label.color.g, label.color.b = color
                label.color.a = 1.0
                prefix = '+' if sign > 0 else '-'
                label.text = f'{prefix}{axis_name} {direction_name}'
                result.markers.append(label)
        self.pub_axes.publish(result)

    def _on_gripper_command(self, msg: Float64):
        if not math.isfinite(msg.data):
            self._warn_throttled('Rejected non-finite gripper command')
            return
        self.gripper_target = float(np.clip(msg.data, -0.174533, 1.74533))

    def _menu_gripper_open(self, _feedback):
        self.gripper_target = float(
            self.get_parameter('gripper_open_position').value)
        self.get_logger().info(
            f'Gripper target: open ({self.gripper_target:.3f} rad)')

    def _menu_gripper_close(self, _feedback):
        self.gripper_target = float(
            self.get_parameter('gripper_closed_position').value)
        self.get_logger().info(
            f'Gripper target: close ({self.gripper_target:.3f} rad)')

    def _menu_emergency_stop(self, _feedback):
        if not self.stop_client.service_is_ready():
            self.get_logger().error('/arm/stop service is not available')
            return
        self.stop_client.call_async(Trigger.Request())
        self.get_logger().warning('Emergency stop requested from RViz')

    def _menu_enable_motion(self, _feedback):
        if not self.enable_client.service_is_ready():
            self.get_logger().error('/arm/enable service is not available')
            return
        request = SetBool.Request()
        request.data = True
        self.enable_client.call_async(request)
        self.get_logger().warning('Hardware motion enable requested from RViz')

    def _menu_disable_motion(self, _feedback):
        if not self.enable_client.service_is_ready():
            self.get_logger().error('/arm/enable service is not available')
            return
        request = SetBool.Request()
        request.data = False
        self.enable_client.call_async(request)
        self.get_logger().warning('Hardware motion disable requested from RViz')

    def _menu_home(self, _feedback):
        request = Trigger.Request()
        self._on_home(request, Trigger.Response())

    def _on_home(self, _request, response):
        if not self.joint_state_synced:
            response.success = False
            response.message = 'waiting for physical joint-state synchronization'
            return response
        if self.have_driver_status and not self.motion_enabled:
            response.success = False
            response.message = 'enable hardware motion before homing'
            return response
        self.q_target = self._home_pose()
        self.target_mat = self.chain.forward_kinematics(
            self.q_target, full_kinematics=True)[-1].copy()
        self.gripper_target = self.home_positions[GRIPPER_JOINT]
        self.target_dirty = False
        self._set_marker_pose_from_target()
        self.get_logger().warning(
            'Homing to geometry-derived orthogonal operational zero')
        response.success = True
        response.message = 'orthogonal operational-zero target accepted'
        return response

    def _circle_tick(self):
        p = self.get_parameter('circle_center').value
        r = float(self.get_parameter('circle_radius').value)
        w = float(self.get_parameter('circle_omega').value)
        dt = self.circle_timer.timer_period_ns * 1e-9
        self._circle_t += w * dt
        msg = PoseStamped()
        msg.header.frame_id = self.get_parameter('base_frame').value
        msg.pose.position.x = float(p[0]) + r * math.cos(self._circle_t)
        msg.pose.position.y = float(p[1]) + r * math.sin(self._circle_t)
        msg.pose.position.z = float(p[2])
        # end-effector z-axis pointing straight down
        msg.pose.orientation.x = 1.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0
        self._on_target(msg)

    # ---- DLS IK patch ----
    def _tip_matrix(self, q):
        """链末端位姿矩阵（用 ikpy 自己的 FK）。"""
        return self.chain.forward_kinematics(np.asarray(q, dtype=float),
                                             full_kinematics=True)[-1]

    def _numeric_jacobian(self, q, eps=2e-4):
        """数值雅可比（3×n，仅位置）。固定关节列为 0，无副作用。"""
        q = np.asarray(q, dtype=float)
        p0 = self._tip_matrix(q)[:3, 3]
        J = np.zeros((3, len(q)))
        for i, act in enumerate(self.chain.active_links_mask):
            if not act or i >= len(q):
                continue
            qp = q.copy()
            qp[i] += eps
            J[:, i] = (self._tip_matrix(qp)[:3, 3] - p0) / eps
        return J

    def _clip_bounds(self, q):
        """按链上各 link 的 bounds 夹紧，并记下被夹的关节与越界量。

        夹紧本身是必要的（解算过程会短暂越界），但**不能悄悄发生**：
        越界量记录下来，由 _solve_ik 统一汇报。否则"机械臂莫名其妙停在
        某个角度"就无从追查。
        """
        for i, link in enumerate(self.chain.links):
            if i >= len(q):
                break
            b = getattr(link, 'bounds', None)
            if b is not None and b[0] is not None and b[1] is not None:
                value = float(q[i])
                lo, hi = float(b[0]), float(b[1])
                if value < lo:
                    over = lo - value
                    name = getattr(link, 'name', f'link{i}')
                    if over > self._clip_worst.get(name, 0.0):
                        self._clip_worst[name] = over
                    q[i] = lo
                elif value > hi:
                    over = value - hi
                    name = getattr(link, 'name', f'link{i}')
                    if over > self._clip_worst.get(name, 0.0):
                        self._clip_worst[name] = over
                    q[i] = hi
        return q

    def _report_clips(self):
        """汇报本轮解算中被限位吃掉的关节（限流，按越界量变化才报）。"""
        if not self._clip_worst:
            return
        changed = {
            name: over for name, over in self._clip_worst.items()
            if over > self._clip_reported.get(name, 0.0) + math.radians(0.05)
        }
        if not changed:
            return
        self._clip_reported.update(self._clip_worst)
        detail = ', '.join(
            f'{name} {math.degrees(over):.1f}°'
            for name, over in sorted(changed.items(),
                                     key=lambda kv: -kv[1]))
        self.get_logger().warning(
            'IK solution hit the joint envelope and was clipped: '
            f'{detail} (worst so far)')

    def _dls_refine(self, q0, target_pos, iters=400, lam=2e-4, step=.8, target_rotation=None):
        """Refine position and the requested orientation together."""
        q = self._clip_bounds(np.asarray(q0, dtype=float).copy())
        weight = .08
        mode = self.orientation_mode if target_rotation is not None else 'none'
        def feature(matrix):
            if mode == 'Z':
                return np.concatenate([matrix[:3,3],weight*matrix[:3,2]])
            if mode == 'all':
                return np.concatenate([matrix[:3,3],weight*matrix[:3,:3].ravel()])
            return matrix[:3,3]
        desired = np.eye(4)
        desired[:3,3] = target_pos
        if target_rotation is not None:
            desired[:3,:3] = target_rotation
        goal = feature(desired)
        for _ in range(iters):
            current = feature(self._tip_matrix(q))
            error = goal-current
            if np.linalg.norm(error[:3]) < 5e-5 and np.linalg.norm(error[3:]) < 4e-5:
                break
            jac = np.zeros((len(error),len(q)))
            for i, active in enumerate(self.chain.active_links_mask):
                if active:
                    shifted = q.copy()
                    shifted[i] += 2e-4
                    jac[:,i] = (feature(self._tip_matrix(shifted))-current)/2e-4
            dq = jac.T @ np.linalg.solve(jac @ jac.T+lam*np.eye(len(error)),error)
            q = self._clip_bounds(q+np.clip(dq*step,-.06,.06))
        return q,float(np.linalg.norm(self._tip_matrix(q)[:3,3]-target_pos))

    def _seed_list(self, ikpy_sol):
        seeds = []
        if ikpy_sol is not None:
            seeds.append(np.asarray(ikpy_sol,dtype=float))
        seeds.extend([self.q_target.copy(),self.q_current.copy(),np.zeros(len(self.chain.links))])
        middle = np.zeros(len(self.chain.links))
        bent = middle.copy()
        ready = dict(zip(IK_JOINT_NAMES,(0.,-1.,1.5,-.7,-2.2)))
        for i,link in enumerate(self.chain.links):
            if link.name in IK_JOINT_NAMES:
                middle[i] = .5*sum(link.bounds)
                bent[i] = ready[link.name]
        seeds.extend([middle,bent])
        return seeds

    def _solve_dls(self, target_mat):
        """Only accept solutions meeting both position and orientation tolerances."""
        mode = None if self.orientation_mode == 'none' else self.orientation_mode
        initial = None
        try:
            initial = self.chain.inverse_kinematics_frame(target_mat,
                         initial_position=self.q_target,orientation_mode=mode)
        except Exception as exc:
            self._warn_throttled(f'ikpy seed failed: {exc}')
        best_q,best_err,best_score = None,float('inf'),float('inf')
        for seed in self._seed_list(initial):
            q,error = self._dls_refine(seed,target_mat[:3,3],target_rotation=target_mat[:3,:3])
            axis_error = self._orientation_error_deg(q,target_mat)
            score = error/self.max_ik_residual+axis_error/5.
            if score < best_score:
                best_q,best_err,best_score = q,error,score
            if error <= self.max_ik_residual and axis_error <= 5.:
                return q,error
        return best_q,best_err

    def _solve_ik(self):
        if self.have_driver_status and not self._hardware_limits_ready:
            self._warn_throttled(
                'Waiting for live driver calibration/limits; target rejected '
                '(the driver has not answered /so101_driver/get_parameters)')
            return
        self._clip_worst.clear()
        sol,error = self._solve_dls(self.target_mat)
        self._report_clips()
        if sol is None or not np.isfinite(sol).all() or not math.isfinite(error) or error > self.max_ik_residual:
            self._warn_throttled(f'Rejected target: position residual {error*1000:.1f}mm')
            return
        angle = self._orientation_error_deg(sol,self.target_mat)
        if not math.isfinite(angle) or angle > 5.:
            self._warn_throttled(f'Rejected target: orientation error {angle:.1f}deg')
            return
        self.q_target = np.asarray(sol,dtype=float)
    # ---- DLS IK patch end ----
    def _tick(self):
        if not self.joint_state_synced:
            return
        if self.target_dirty:
            self._solve_ik()
            # A rejected target stays rejected until the operator changes it;
            # never repeatedly optimize a static redundant-position target.
            self.target_dirty = False
        max_step = self.max_speed / self.rate
        delta = self.q_target - self.q_current
        np.clip(delta, -max_step, max_step, out=delta)
        self.q_current += delta

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        link_names = [getattr(link, 'name', None) for link in self.chain.links]
        positions = []
        for name in JOINT_NAMES:
            if name == GRIPPER_JOINT:
                positions.append(self.gripper_target)
                continue
            positions.append(float(self.q_current[link_names.index(name)]))
        js.position = positions
        self.pub_command.publish(js)

    def _orientation_error_deg(self,q,target):
        if self.orientation_mode == 'none':
            return 0.
        actual = self._tip_matrix(q)[:3,:3]
        desired = target[:3,:3]
        if self.orientation_mode == 'Z':
            cosine = np.dot(actual[:,2],desired[:,2])
        else:
            cosine = (np.trace(desired @ actual.T)-1)/2
        return math.degrees(math.acos(float(np.clip(cosine,-1.,1.))))

    def _request_hardware_limits(self):
        if self._limits_pending or not self._limits_client.service_is_ready():
            return
        self._limits_pending = True
        request = GetParameters.Request(names=['mode','zero_raw','direction','raw_min','raw_max'])
        future = self._limits_client.call_async(request)
        future.add_done_callback(self._receive_hardware_limits)

    def _receive_hardware_limits(self,future):
        self._limits_pending = False
        try:
            values = future.result().values
            mode = values[0].string_value
            arrays = [list(v.integer_array_value) for v in values[1:]]
            if mode not in ('sim','direct','lerobot') or any(len(a) != 6 for a in arrays):
                raise ValueError('driver parameters missing or invalid')
            zero,direction,minimum,maximum = arrays
            bounds = {}
            for name,z,d,lo,hi in zip(JOINT_NAMES,zero,direction,minimum,maximum):
                if d not in (-1,1) or not 0 <= lo < hi <= 4095:
                    raise ValueError('invalid servo limits')
                ends = [(lo-z)*d*2*math.pi/4096,(hi-z)*d*2*math.pi/4096]
                bounds[name] = (min(ends),max(ends))
            rows = []
            signature = []
            for link in self.chain.links:
                if link.name in self._urdf_joint_bounds:
                    original = self._urdf_joint_bounds[link.name]
                    if mode == 'sim':
                        link.bounds = original
                        self._limits_source[link.name] = 'urdf (sim)'
                    else:
                        lo,hi = bounds[link.name]
                        link.bounds = (max(original[0],lo),min(original[1],hi))
                        if link.bounds[0] >= link.bounds[1]:
                            raise ValueError('driver and URDF ranges do not overlap')
                        # Which side is actually binding tells the operator
                        # whether a refused pose is a model limit or a
                        # mechanical one.
                        low_src = 'driver' if lo > original[0] + 1e-9 else 'urdf'
                        high_src = 'driver' if hi < original[1] - 1e-9 else 'urdf'
                        self._limits_source[link.name] = f'{low_src}/{high_src}'
                    rows.append(
                        f'  {link.name:<14} '
                        f'[{math.degrees(link.bounds[0]):+7.2f}, '
                        f'{math.degrees(link.bounds[1]):+7.2f}] deg'
                        f'   low/high bound by {self._limits_source[link.name]}')
                    signature.append((link.name, round(link.bounds[0], 6),
                                      round(link.bounds[1], 6)))
            self._hardware_limits_ready = True
            # Log only when the envelope actually changes; this callback fires
            # on a 1 Hz timer.
            if signature != self._limits_signature:
                self._limits_signature = signature
                self.get_logger().info(
                    f'Effective joint envelope (URDF ∩ driver, mode={mode}):\n'
                    + '\n'.join(rows))
        except Exception as exc:
            self._hardware_limits_ready = False
            self._warn_throttled(f'Cannot validate driver limits: {exc}')

    def _warn_throttled(self, text):
        now = time.monotonic()
        if now - self.last_warn > 2.0:
            self.last_warn = now
            self.get_logger().warning(text)


def main(args=None):
    rclpy.init(args=args)
    node = So101IkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
