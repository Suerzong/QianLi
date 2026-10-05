#!/usr/bin/env python3
"""Run deterministic QianLi navigation jobs and write evidence, including failures.

All coordinates are metres in the scene/world coordinate system. The generated
map must share this coordinate system. Smoke goals run continuously from spawn;
full-suite jobs first navigate to their declared starts, never teleport.

Collision metrics use the requested octagonal plate footprint in XY, against
the manifest's projected obstacle boxes. They exclude wheel protrusion and do
not establish physical contact accuracy in an ideal kinematic simulator.
"""

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback


RAW_FOOTPRINT = [(.35, .20), (.20, .35), (-.20, .35), (-.35, .20),
                 (-.35, -.20), (-.20, -.35), (.20, -.35), (.35, -.20)]
PADDING = .06


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def offset_polygon(vertices, distance):
    """Exact outward offset of a convex CCW polygon via adjacent offset lines."""
    lines = []
    for a, b in zip(vertices, vertices[1:] + vertices[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        norm = math.hypot(dx, dy)
        nx, ny = dy / norm, -dx / norm
        lines.append((nx, ny, nx * a[0] + ny * a[1] + distance))
    result = []
    for i, current in enumerate(lines):
        previous = lines[i - 1]
        ax, ay, ac = previous
        bx, by, bc = current
        determinant = ax * by - ay * bx
        result.append(((ac * by - ay * bc) / determinant,
                       (ax * bc - ac * bx) / determinant))
    return result


PADDED_FOOTPRINT = offset_polygon(RAW_FOOTPRINT, PADDING)


def transform_polygon(vertices, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * vx - s * vy, y + s * vx + c * vy)
            for vx, vy in vertices]


def polygon_bounds(vertices):
    return (min(v[0] for v in vertices), min(v[1] for v in vertices),
            max(v[0] for v in vertices), max(v[1] for v in vertices))


def aabb_distance(a, b):
    dx = max(a[0] - b[2], b[0] - a[2], 0.)
    dy = max(a[1] - b[3], b[1] - a[3], 0.)
    return math.hypot(dx, dy)


def sat_penetration(a, b):
    """None when disjoint; minimum overlap on all separating-axis candidates."""
    overlap = math.inf
    for polygon in (a, b):
        for p, q in zip(polygon, polygon[1:] + polygon[:1]):
            dx, dy = q[0] - p[0], q[1] - p[1]
            length = math.hypot(dx, dy)
            nx, ny = -dy / length, dx / length
            pa = [x * nx + y * ny for x, y in a]
            pb = [x * nx + y * ny for x, y in b]
            amin, amax, bmin, bmax = min(pa), max(pa), min(pb), max(pb)
            if amax < bmin - 1e-9 or bmax < amin - 1e-9:
                return None
            # Account for containment: the required translation is to either
            # side, not merely the length of the common projection interval.
            overlap = min(overlap, amax - bmin, bmax - amin)
    return max(0., overlap)


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    fraction = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)
    fraction = max(0., min(1., fraction))
    return math.hypot(p[0] - a[0] - fraction * dx,
                      p[1] - a[1] - fraction * dy)


def polygon_clearance(a, b):
    penetration = sat_penetration(a, b)
    if penetration is not None:
        return -penetration, True
    distance = math.inf
    for polygon, other in ((a, b), (b, a)):
        for p in polygon:
            for q, r in zip(other, other[1:] + other[:1]):
                distance = min(distance, point_segment_distance(p, q, r))
    return distance, False


def prepare_obstacles(manifest):
    obstacles = []
    for i, box in enumerate(manifest.get('obstacles', [])):
        sx, sy = float(box['sx']), float(box['sy'])
        if sx <= 0 or sy <= 0:
            raise ValueError('Obstacle {} has non-positive XY size'.format(box.get('id', i)))
        polygon = transform_polygon(
            [(-sx / 2, -sy / 2), (sx / 2, -sy / 2),
             (sx / 2, sy / 2), (-sx / 2, sy / 2)],
            (float(box['x']), float(box['y']), float(box.get('yaw', 0.))))
        obstacles.append({'id': str(box.get('id', i)), 'polygon': polygon,
                          'bounds': polygon_bounds(polygon)})
    if not obstacles:
        raise ValueError('Manifest must contain obstacles for geometry checks')
    return obstacles


def footprint_clearance(polygon, obstacles):
    bounds = polygon_bounds(polygon)
    nearest, nearest_id, intersecting = math.inf, None, []
    # Closest AABBs first substantially reduce exact polygon work in big rooms.
    candidates = sorted((aabb_distance(bounds, o['bounds']), i)
                        for i, o in enumerate(obstacles))
    for lower_bound, index in candidates:
        if lower_bound > max(nearest, 0.):
            break
        obstacle = obstacles[index]
        distance, overlaps = polygon_clearance(polygon, obstacle['polygon'])
        if overlaps:
            intersecting.append(obstacle['id'])
        if distance < nearest:
            nearest, nearest_id = distance, obstacle['id']
    return nearest, nearest_id, intersecting


def finite(value):
    return value if value is not None and math.isfinite(value) else None


def write_report(path, report):
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + '.tmp')
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n', encoding='utf-8')
    os.replace(temporary, destination)


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    return values[min(len(values) - 1, int(math.ceil(fraction * len(values))) - 1)]


def metrics_new(pose, sim_time):
    return {'path_length_m': 0., 'ground_truth_samples': 0,
            'geometry_samples': 0, 'raw_collision_samples': 0,
            'raw_collision_episodes': 0, 'padded_collision_samples': 0,
            'min_raw_clearance_m': None, 'min_padded_clearance_m': None,
            'closest_raw_obstacle': None, 'closest_padded_obstacle': None,
            'trajectory': [], 'feedback': [], 'amcl_matches': 0,
            'amcl_position_error_rmse_m': None, 'amcl_position_error_p95_m': None,
            'amcl_yaw_error_rmse_rad': None, 'amcl_yaw_error_p95_rad': None,
            'start_physical_pose': list(pose) if pose else None,
            'start_sim_time_s': sim_time, 'end_sim_time_s': None,
            'large_pose_steps': [], '_last_pose': pose, '_last_raw_collision': False,
            '_last_geometry_wall': 0., '_last_trajectory_wall': 0.,
            '_position_errors': [], '_yaw_errors': []}


def metrics_public(metrics):
    result = {k: v for k, v in metrics.items() if not k.startswith('_')}
    position, yaw = metrics['_position_errors'], metrics['_yaw_errors']
    result['amcl_matches'] = len(position)
    if position:
        result['amcl_position_error_rmse_m'] = math.sqrt(sum(v * v for v in position) / len(position))
        result['amcl_position_error_p95_m'] = percentile(position, .95)
        result['amcl_yaw_error_rmse_rad'] = math.sqrt(sum(v * v for v in yaw) / len(yaw))
        result['amcl_yaw_error_p95_rad'] = percentile(yaw, .95)
    return result


def create_node(manifest, obstacles, options):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import NavigateToPose
    from rosgraph_msgs.msg import Clock

    class BenchmarkNode(Node):
        def __init__(self):
            super().__init__('qianli_teaching_building_benchmark',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.client = ActionClient(self, NavigateToPose, options.action)
            self.stop_publisher = self.create_publisher(TwistStamped, options.cmd_vel, 10)
            self.pose, self.sim_time = None, None
            self.gt_received_wall = 0.
            self.gt_frame, self.amcl_frame = None, None
            self.gt_history = deque(maxlen=2500)
            self.active_metrics, self.active_handle = None, None
            self.pending_send = None
            self.last_feedback_wall = 0.
            self.safety_stop = False
            self.global_metrics = metrics_new(None, None)
            self.create_subscription(Odometry, options.ground_truth, self.on_ground_truth,
                                     qos_profile_sensor_data)
            self.create_subscription(PoseWithCovarianceStamped, options.amcl,
                                     self.on_amcl, qos_profile_sensor_data)
            self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)

        def on_clock(self, message):
            self.sim_time = message.clock.sec + message.clock.nanosec * 1e-9

        def on_ground_truth(self, message):
            p = message.pose.pose
            pose = (p.position.x, p.position.y, quaternion_yaw(p.orientation))
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.pose, self.gt_frame = pose, message.header.frame_id
            now = time.monotonic()
            self.gt_received_wall = now
            self.gt_history.append((stamp, pose))
            for metrics in (self.global_metrics, self.active_metrics):
                if metrics is None:
                    continue
                previous = metrics['_last_pose']
                metrics['ground_truth_samples'] += 1
                if previous is not None:
                    step = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
                    metrics['path_length_m'] += step
                    if step > 2.:
                        metrics['large_pose_steps'].append({'distance_m': step,
                                                            'sim_time_s': self.sim_time})
                metrics['_last_pose'] = pose
                metrics['end_sim_time_s'] = self.sim_time
                if metrics['start_physical_pose'] is None:
                    metrics['start_physical_pose'] = list(pose)
                    metrics['start_sim_time_s'] = self.sim_time
                if now - metrics['_last_trajectory_wall'] >= .20:
                    metrics['trajectory'].append({'pose': list(pose),
                                                   'sim_time_s': self.sim_time})
                    metrics['_last_trajectory_wall'] = now
            if now - self.global_metrics['_last_geometry_wall'] < options.geometry_interval:
                return
            raw, raw_id, raw_hits = footprint_clearance(transform_polygon(RAW_FOOTPRINT, pose), obstacles)
            padded, padded_id, padded_hits = footprint_clearance(transform_polygon(PADDED_FOOTPRINT, pose), obstacles)
            for metrics in (self.global_metrics, self.active_metrics):
                if metrics is None:
                    continue
                metrics['_last_geometry_wall'] = now
                metrics['geometry_samples'] += 1
                if metrics['min_raw_clearance_m'] is None or raw < metrics['min_raw_clearance_m']:
                    metrics['min_raw_clearance_m'], metrics['closest_raw_obstacle'] = finite(raw), raw_id
                if metrics['min_padded_clearance_m'] is None or padded < metrics['min_padded_clearance_m']:
                    metrics['min_padded_clearance_m'], metrics['closest_padded_obstacle'] = finite(padded), padded_id
                metrics['raw_collision_samples'] += bool(raw_hits)
                metrics['padded_collision_samples'] += bool(padded_hits)
                if raw_hits and not metrics['_last_raw_collision']:
                    metrics['raw_collision_episodes'] += 1
                metrics['_last_raw_collision'] = bool(raw_hits)
            if raw_hits and self.active_metrics is not None:
                self.safety_stop = True

        def on_amcl(self, message):
            if not self.gt_history:
                return
            self.amcl_frame = message.header.frame_id
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            gt_stamp, gt_pose = (min(self.gt_history, key=lambda item: abs(item[0] - stamp))
                                 if stamp else self.gt_history[-1])
            if stamp and abs(gt_stamp - stamp) > options.pose_sync_tolerance:
                return
            p = message.pose.pose
            error = math.hypot(p.position.x - gt_pose[0], p.position.y - gt_pose[1])
            yaw_error = abs(wrap_angle(quaternion_yaw(p.orientation) - gt_pose[2]))
            for metrics in (self.global_metrics, self.active_metrics):
                if metrics is not None:
                    metrics['_position_errors'].append(error)
                    metrics['_yaw_errors'].append(yaw_error)

        def stop(self):
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            self.stop_publisher.publish(message)

        def cancel(self):
            if self.pending_send is not None:
                pending = self.pending_send
                deadline = time.monotonic() + 2.
                while rclpy.ok() and not pending.done() and time.monotonic() < deadline:
                    self.stop()
                    rclpy.spin_once(self, timeout_sec=.05)
                if pending.done():
                    try:
                        handle = pending.result()
                        if handle.accepted:
                            self.active_handle = handle
                    except Exception:
                        pass
                else:
                    pending.add_done_callback(self.cancel_late_acceptance)
                self.pending_send = None
            if self.active_handle is not None:
                future = self.active_handle.cancel_goal_async()
                deadline = time.monotonic() + 2.
                while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                    self.stop()
                    rclpy.spin_once(self, timeout_sec=.05)
                self.active_handle = None
            self.stop()

        def cancel_late_acceptance(self, future):
            try:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
                self.stop()
            except Exception:
                pass

        def feedback(self, message):
            if self.active_metrics is None:
                return
            now = time.monotonic()
            if now - self.last_feedback_wall < 1.:
                return
            self.last_feedback_wall = now
            feedback = message.feedback
            entry = {'sim_time_s': self.sim_time}
            for name in ('distance_remaining', 'number_of_recoveries'):
                if hasattr(feedback, name):
                    entry[name] = getattr(feedback, name)
            for name in ('navigation_time', 'estimated_time_remaining'):
                if hasattr(feedback, name):
                    value = getattr(feedback, name)
                    entry[name + '_s'] = value.sec + value.nanosec * 1e-9
            self.active_metrics['feedback'].append(entry)
            print('feedback', json.dumps(entry), flush=True)

        def navigate(self, goal_pose, task_id):
            self.active_metrics = metrics_new(self.pose, self.sim_time)
            self.safety_stop = False
            if self.pose is not None:
                raw, raw_id, raw_hits = footprint_clearance(transform_polygon(RAW_FOOTPRINT, self.pose), obstacles)
                padded, padded_id, padded_hits = footprint_clearance(transform_polygon(PADDED_FOOTPRINT, self.pose), obstacles)
                self.active_metrics.update({'geometry_samples': 1,
                                            'min_raw_clearance_m': finite(raw),
                                            'min_padded_clearance_m': finite(padded),
                                            'closest_raw_obstacle': raw_id,
                                            'closest_padded_obstacle': padded_id,
                                            'raw_collision_samples': int(bool(raw_hits)),
                                            'raw_collision_episodes': int(bool(raw_hits)),
                                            'padded_collision_samples': int(bool(padded_hits)),
                                            '_last_raw_collision': bool(raw_hits)})
                self.safety_stop = bool(raw_hits)
            self.last_feedback_wall = 0.
            started = time.monotonic()
            result = {'id': task_id, 'goal': list(goal_pose), 'action_status': None,
                      'action_status_name': None, 'terminal_reason': None,
                      'accepted': False, 'final_physical_pose': None,
                      'final_position_error_m': None, 'final_yaw_error_rad': None}
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = options.map_frame
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(goal_pose[0]), float(goal_pose[1])
            goal.pose.pose.orientation.z = math.sin(goal_pose[2] / 2)
            goal.pose.pose.orientation.w = math.cos(goal_pose[2] / 2)
            statuses = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING',
                        3: 'CANCELING', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
            print('goal', task_id, list(goal_pose), flush=True)
            try:
                if self.safety_stop:
                    result['terminal_reason'] = 'raw_footprint_collision'
                    return result
                send = self.client.send_goal_async(goal, feedback_callback=self.feedback)
                self.pending_send = send
                while rclpy.ok() and not send.done():
                    if time.monotonic() - started > min(options.timeout, 15.):
                        result['terminal_reason'] = 'goal_acceptance_timeout'
                        # An eventual acceptance must not leave an orphan goal.
                        self.cancel()
                        return result
                    rclpy.spin_once(self, timeout_sec=.10)
                if not rclpy.ok():
                    result['terminal_reason'] = 'ros_context_stopped'
                    return result
                handle = send.result()
                self.pending_send = None
                if not handle.accepted:
                    result['terminal_reason'] = 'goal_rejected'
                    return result
                self.active_handle = handle
                result['accepted'] = True
                complete = handle.get_result_async()
                while rclpy.ok() and not complete.done():
                    reason = None
                    if self.safety_stop:
                        reason = 'raw_footprint_collision'
                    elif time.monotonic() - self.gt_received_wall > options.telemetry_timeout:
                        reason = 'ground_truth_stream_stale'
                    elif time.monotonic() - started > options.timeout:
                        reason = 'wall_timeout'
                    if reason:
                        result['terminal_reason'] = reason
                        self.cancel()
                        # Cancellation is asynchronous: retain its real terminal
                        # action status when delivered, rather than inventing it.
                        deadline = time.monotonic() + 2.
                        while rclpy.ok() and not complete.done() and time.monotonic() < deadline:
                            self.stop()
                            rclpy.spin_once(self, timeout_sec=.05)
                        break
                    rclpy.spin_once(self, timeout_sec=.10)
                if complete.done():
                    completed = complete.result()
                    result['action_status'] = int(completed.status)
                    result['action_status_name'] = statuses.get(int(completed.status), 'UNKNOWN')
                    if result['terminal_reason'] is None:
                        result['terminal_reason'] = result['action_status_name'].lower()
                    action_result = completed.result
                    for name in ('error_code', 'error_msg'):
                        if hasattr(action_result, name):
                            result[name] = getattr(action_result, name)
                elif result['terminal_reason'] is None:
                    result['terminal_reason'] = 'ros_context_stopped'
                self.active_handle = None
                return result
            except (KeyboardInterrupt, InterruptedError):
                result['terminal_reason'] = 'interrupted'
                raise
            except Exception as error:
                result['terminal_reason'] = 'exception'
                result['exception'] = '{}: {}'.format(type(error).__name__, error)
                self.cancel()
                return result
            finally:
                result['elapsed_wall_s'] = time.monotonic() - started
                if self.pose is not None:
                    result['final_physical_pose'] = list(self.pose)
                    result['final_position_error_m'] = math.hypot(self.pose[0] - goal_pose[0], self.pose[1] - goal_pose[1])
                    result['final_yaw_error_rad'] = abs(wrap_angle(self.pose[2] - goal_pose[2]))
                result['metrics'] = metrics_public(self.active_metrics)
                self.last_partial_result = result
                self.active_metrics = None

    return BenchmarkNode()


def accepted(result, expected, options):
    metrics = result.get('metrics', {})
    no_collision = (metrics.get('raw_collision_samples', 1) == 0
                    and metrics.get('geometry_samples', 0) > 0)
    padded_safe = metrics.get('padded_collision_samples', 1) == 0
    if expected == 'unreachable':
        from nav2_msgs.action import ComputePathToPose
        aborted = (result.get('terminal_reason') == 'aborted'
                   and result.get('action_status') == 6
                   and result.get('error_code') == ComputePathToPose.Result.NO_VALID_PATH)
        return aborted and no_collision
    return (result.get('action_status') == 4 and no_collision and padded_safe
            and result.get('final_position_error_m', math.inf) <= options.position_tolerance
            and result.get('final_yaw_error_rad', math.inf) <= options.yaw_tolerance)


def main(argv=None):
    global RAW_FOOTPRINT, PADDING, PADDED_FOOTPRINT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--suite', choices=('smoke', 'full'), default='smoke')
    parser.add_argument('--timeout', type=float, default=240., help='Per navigation phase, wall seconds')
    parser.add_argument('--startup-timeout', type=float, default=60.)
    parser.add_argument('--telemetry-timeout', type=float, default=3.)
    parser.add_argument('--geometry-interval', type=float, default=.10, help='Wall seconds between footprint checks')
    parser.add_argument('--pose-sync-tolerance', type=float, default=.20, help='AMCL/GT stamp match in sim seconds')
    parser.add_argument('--position-tolerance', type=float, default=.25)
    parser.add_argument('--yaw-tolerance', type=float, default=.262)
    parser.add_argument('--start-tolerance', type=float, default=.35)
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--ground-truth', default='/simulation/ground_truth')
    parser.add_argument('--amcl', default='/amcl_pose')
    parser.add_argument('--cmd-vel', default='/cmd_vel')
    parser.add_argument('--action', default='/navigate_to_pose')
    options = parser.parse_args(argv)
    report = {'schema_version': 1, 'suite': options.suite,
              'manifest': str(Path(options.manifest).resolve()), 'status': 'starting',
              'started_unix_s': time.time(), 'options': vars(options), 'tasks': [],
              'footprint': RAW_FOOTPRINT, 'padding_m': PADDING,
              'padded_footprint': PADDED_FOOTPRINT,
              'limitations': ['Synthetic scene; map and world coordinate systems must coincide.',
                              'Geometry checks cover octagonal plate footprint, excluding wheel protrusion.',
                              'Ideal kinematic motion is not measured physical wheel odometry.',
                              'Clearance is sampled; negative clearance is SAT penetration depth.',
                              'Furniture is projected to XY as the manifest obstacle boxes.']}
    write_report(options.output, report)
    node, ros = None, None
    active_task = None
    exit_code = 1
    try:
        manifest = json.loads(Path(options.manifest).read_text(encoding='utf-8'))
        RAW_FOOTPRINT = manifest['footprint']
        PADDING = float(manifest['parameters']['footprint_padding'])
        PADDED_FOOTPRINT = offset_polygon(RAW_FOOTPRINT, PADDING)
        report.update(footprint=RAW_FOOTPRINT, padding_m=PADDING, padded_footprint=PADDED_FOOTPRINT)
        jobs = manifest.get('smoke_tasks' if options.suite == 'smoke' else 'tasks', [])
        if not jobs:
            raise ValueError('Requested suite has no tasks in manifest')
        obstacles = prepare_obstacles(manifest)
        report['obstacle_count'] = len(obstacles)
        report['manifest_spawn'] = manifest.get('spawn')
        import rclpy as ros
        from rclpy.signals import SignalHandlerOptions
        ros.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        def interrupted(_signum, _frame):
            raise InterruptedError('Termination signal')
        signal.signal(signal.SIGTERM, interrupted)
        node = create_node(manifest, obstacles, options)
        deadline = time.monotonic() + options.startup_timeout
        while ros.ok() and time.monotonic() < deadline:
            ready = node.pose is not None and node.sim_time is not None and node.client.server_is_ready()
            fresh = time.monotonic() - node.gt_received_wall < options.telemetry_timeout
            if ready and fresh:
                break
            ros.spin_once(node, timeout_sec=.10)
        else:
            raise TimeoutError('Startup requires fresh physical ground truth, /clock and NavigateToPose server')
        report['initial_physical_pose'] = list(node.pose)
        if options.suite == 'smoke':
            expected_start = jobs[0]['start']
            if (math.hypot(node.pose[0]-expected_start[0], node.pose[1]-expected_start[1]) > options.start_tolerance
                    or abs(wrap_angle(node.pose[2]-expected_start[2])) > options.yaw_tolerance):
                raise RuntimeError('Deterministic smoke requires the manifest spawn pose; restart the scene before repeating it')
        report['status'] = 'running'
        write_report(options.output, report)
        for job in jobs:
            active_task = {'id': str(job['id']), 'expected': job.get('expected', 'reachable'),
                           'goal': list(job['goal']), 'status': 'running'}
            report['tasks'].append(active_task)
            write_report(options.output, report)
            if options.suite == 'full' and job.get('start'):
                start = job['start']
                start_error = math.hypot(node.pose[0] - start[0], node.pose[1] - start[1])
                start_yaw_error = abs(wrap_angle(node.pose[2] - start[2]))
                if start_error > options.start_tolerance or start_yaw_error > options.yaw_tolerance:
                    reposition = node.navigate(start, str(job['id']) + '/reposition')
                    active_task['reposition'] = reposition
                    if not accepted(reposition, 'reachable', options):
                        active_task['status'] = 'skipped_start_unavailable'
                        active_task['passed'] = False
                        write_report(options.output, report)
                        if reposition['terminal_reason'] in ('raw_footprint_collision', 'ground_truth_stream_stale'):
                            report['status'] = 'stopped_for_safety_or_telemetry'
                            break
                        continue
            result = node.navigate(job['goal'], str(job['id']))
            active_task.update(result)
            active_task['passed'] = accepted(result, active_task['expected'], options)
            active_task['status'] = 'passed' if active_task['passed'] else 'failed'
            write_report(options.output, report)
            if result['terminal_reason'] in ('raw_footprint_collision', 'ground_truth_stream_stale',
                                              'ros_context_stopped', 'goal_acceptance_timeout'):
                report['status'] = 'stopped_for_safety_or_telemetry'
                break
        else:
            report['status'] = 'completed'
        report['passed_tasks'] = sum(t.get('passed', False) for t in report['tasks'])
        report['requested_tasks'] = len(jobs)
        exit_code = 0 if report['status'] == 'completed' and report['passed_tasks'] == len(jobs) else 1
    except (KeyboardInterrupt, InterruptedError) as error:
        report['status'] = 'interrupted'
        report['exception'] = str(error)
        if active_task is not None and node is not None and hasattr(node, 'last_partial_result'):
            if node.last_partial_result['id'] == active_task['id']:
                active_task.update(node.last_partial_result)
            else:
                active_task['interrupted_phase_result'] = node.last_partial_result
            active_task['status'] = 'interrupted'
            active_task['passed'] = False
        exit_code = 130
    except Exception as error:
        report['status'] = 'failed'
        report['exception'] = '{}: {}'.format(type(error).__name__, error)
        report['traceback'] = traceback.format_exc()
        if active_task is not None:
            active_task['status'] = 'failed'
            active_task['passed'] = False
        print(report['exception'], file=sys.stderr, flush=True)
    finally:
        if node is not None:
            try:
                node.cancel()
                stop_deadline = time.monotonic() + .50
                while ros.ok() and time.monotonic() < stop_deadline:
                    node.stop()
                    ros.spin_once(node, timeout_sec=.05)
            except Exception as error:
                report['stop_exception'] = str(error)
            report['global_metrics'] = metrics_public(node.global_metrics)
            report['ground_truth_frame'] = node.gt_frame
            report['amcl_frame'] = node.amcl_frame
            try:
                node.destroy_node()
            except Exception as error:
                report['destroy_exception'] = str(error)
        if ros is not None:
            try:
                if ros.ok():
                    ros.shutdown()
            except Exception as error:
                report['shutdown_exception'] = str(error)
        report['finished_unix_s'] = time.time()
        report['exit_code'] = exit_code
        write_report(options.output, report)
        print('report', options.output, report['status'], flush=True)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
