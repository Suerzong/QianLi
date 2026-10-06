#!/usr/bin/env python3
"""Explore live SLAM maps by serially delegating safe goals to Nav2."""
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from frontier_core import Grid, extract, path_is_safe, safe_cells


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('qianli_frontier_explorer')
        defaults = {
            'enabled': True, 'base_frame': 'base_footprint', 'map_frame': 'map',
            'robot_radius': .47, 'free_threshold': 20, 'min_frontier_length': .35,
            'approach_distance': 1.5, 'min_goal_distance': .55, 'goal_clearance_margin': .1,
            'selection_interval_s': 2., 'sensor_timeout_s': 5., 'map_timeout_s': 15., 'tf_timeout_s': 2.,
            'plan_timeout_s': 15., 'goal_timeout_s': 150., 'progress_timeout_s': 45.,
            'blacklist_duration_s': 120., 'revisit_delay_s': 40., 'blacklist_radius': .65,
            'settle_time_s': 2., 'stagnation_timeout_s': 150., 'growth_cells': 100,
            'max_duration_s': 900., 'initial_spin': True, 'initial_spin_angle': 1.57,
            'stable_empty_rounds': 3, 'report_file': '', 'max_candidates': 32,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if self.p['robot_radius'] <= 0 or self.p['selection_interval_s'] <= 0:
            raise ValueError('robot_radius and selection_interval_s must be positive')
        self.enabled = self.p['enabled']
        self.grid = None
        self.known_cells = self.initial_known_cells = self.growth_reference = 0
        self.last_map_wall = self.last_scan_wall = self.last_growth_wall = 0.
        self.scan_valid = False
        self.start_wall = self.enabled_wall = time.monotonic()
        self.last_analysis_wall = self.last_publish_wall = 0.
        self.next_selection_wall = 0.
        self.empty_rounds = 0
        self.bootstrap_failures = 0
        self.bootstrap_done = not self.p['initial_spin']
        self.phase, self.reason, self.terminal = 'waiting', 'waiting_for_sensors', ''
        self.operation = None
        self.blacklist = []
        self.analysis = None
        self.history = []
        self.successes = self.failures = self.map_resets = 0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.lifecycle_clients = {name: self.create_client(GetState, '/'+name+'/get_state')
                                  for name in ('planner_server', 'controller_server', 'bt_navigator', 'behavior_server')}
        self.lifecycle_states = {}
        self.lifecycle_pending = {}
        self.last_lifecycle_query = 0.
        self.nav_clients = {
            'plan': ActionClient(self, ComputePathToPose, '/compute_path_to_pose'),
            'navigate': ActionClient(self, NavigateToPose, '/navigate_to_pose'),
            'spin': ActionClient(self, Spin, '/spin'),
        }
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/exploration/status', latch)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', latch)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latch)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(SetBool, '/exploration/enable', self.on_enable)
        self.timer = self.create_timer(.5, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def event(self, name, **values):
        entry = {'event': name, 'wall_elapsed_s': round(time.monotonic()-self.start_wall, 3), **values}
        self.history.append(entry)
        self.get_logger().info(json.dumps(entry, ensure_ascii=False))

    def on_map(self, msg):
        if msg.header.frame_id != self.p['map_frame'] or msg.info.resolution <= 0:
            return
        if len(msg.data) != msg.info.width*msg.info.height or not msg.data:
            return
        cells = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        new = Grid(cells, msg.info.resolution, msg.info.origin.position.x,
                   msg.info.origin.position.y, yaw(msg.info.origin.orientation))
        count = int((cells >= 0).sum())
        if self.grid is not None and (abs(new.resolution-self.grid.resolution) > 1e-6 or
                                     abs(new.origin_yaw-self.grid.origin_yaw) > 1e-4 or
                                     (self.known_cells >= 100 and count < self.known_cells*.5)):
            self.map_resets += 1
            self.empty_rounds = 0
            self.bootstrap_failures = 0
            self.blacklist.clear()
            self.bootstrap_done = not self.p['initial_spin']
            self.initial_known_cells = self.growth_reference = count
            self.last_growth_wall = time.monotonic()
            self.cancel('map_reset')
            self.event('map_reset')
        self.grid, self.known_cells = new, count
        self.last_map_wall = time.monotonic()
        if not self.initial_known_cells:
            self.initial_known_cells = self.growth_reference = count
            self.last_growth_wall = self.last_map_wall
        if count >= self.growth_reference+self.p['growth_cells']:
            self.growth_reference = count
            self.last_growth_wall = self.last_map_wall

    def on_scan(self, msg):
        self.last_scan_wall = time.monotonic()
        valid = sum(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges)
        self.scan_valid = bool(msg.ranges) and valid >= max(3, math.ceil(len(msg.ranges)*.05))

    def pose(self):
        try:
            stamped = self.buffer.lookup_transform(self.p['map_frame'], self.p['base_frame'], rclpy.time.Time())
            age = (self.get_clock().now()-rclpy.time.Time.from_msg(stamped.header.stamp)).nanoseconds*1e-9
            if age > self.p['tf_timeout_s'] or age < -.5:
                return None
            t = stamped.transform
            return t.translation.x, t.translation.y, yaw(t.rotation)
        except TransformException:
            return None

    def healthy(self):
        now = time.monotonic()
        if self.grid is None or now-self.last_map_wall > self.p['map_timeout_s']:
            return 'map_missing_or_stale'
        if not self.scan_valid or now-self.last_scan_wall > self.p['sensor_timeout_s']:
            return 'scan_invalid_or_stale'
        if self.pose() is None:
            return 'tf_unavailable'
        return ''

    def on_enable(self, request, response):
        if request.data and self.operation is not None and self.operation['cancel']:
            response.success, response.message = False, 'Wait for cancellation confirmation in /exploration/status'
            return response
        if request.data:
            self.enabled, self.terminal, self.phase = True, '', 'waiting'
            self.enabled_wall = time.monotonic()
            self.last_growth_wall = self.enabled_wall
            self.empty_rounds = 0
            self.event('enabled')
        else:
            self.stop('stopped')
        response.success = True
        response.message = 'Enabled' if request.data else 'Stop requested; status confirms action termination'
        return response

    def stop(self, reason):
        self.enabled, self.terminal = False, reason
        if self.operation is not None:
            self.cancel(reason)
        else:
            self.phase, self.reason = reason, reason
        self.publish(force=True)

    def pose_message(self, candidate):
        msg = PoseStamped()
        msg.header.frame_id = self.p['map_frame']
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = candidate.x, candidate.y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(candidate.yaw/2), math.cos(candidate.yaw/2)
        return msg

    def send(self, kind, goal, candidate=None):
        if self.operation is not None:
            raise RuntimeError('Only one Nav2 operation may be outstanding')
        now = time.monotonic()
        op = {'kind': kind, 'candidate': candidate, 'handle': None, 'cancel': '',
              'cancel_sent': False, 'started': now, 'progress_wall': now, 'progress_pose': self.pose()}
        self.operation = op
        self.phase, self.reason = kind, ''
        future = self.nav_clients[kind].send_goal_async(goal)
        op['send_future'] = future
        future.add_done_callback(lambda f: self.goal_reply(op, f))

    def goal_reply(self, op, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.finish(op, None, 'goal_rejected')
                return
            op['handle'] = handle
            op['result_future'] = handle.get_result_async()
            op['result_future'].add_done_callback(lambda f: self.result_reply(op, f))
            if op['cancel']:
                self.cancel(op['cancel'])
        except Exception as exc:
            self.finish(op, None, 'action_error: '+str(exc))

    def result_reply(self, op, future):
        try:
            self.finish(op, future.result(), '')
        except Exception as exc:
            self.get_logger().error('Action result processing failed: '+str(exc))
            if self.operation is op:
                self.finish(op, None, 'result_error: '+str(exc))
            else:
                self.stop('internal_error')

    def cancel(self, reason):
        op = self.operation
        if op is None:
            return
        op['cancel'] = op['cancel'] or reason
        self.phase, self.reason = 'canceling', op['cancel']
        if op['handle'] is not None and not op['cancel_sent']:
            op['cancel_sent'] = True
            op['cancel_future'] = op['handle'].cancel_goal_async()
            self.event('cancel_requested', kind=op['kind'], reason=op['cancel'])
        # A late accepted goal is canceled in goal_reply. Never send its successor
        # until this goal's RESULT confirms termination, regardless of cancel ACK.

    def reject_candidate(self, candidate, reason, retry_delay=None):
        self.failures += 1
        duration = self.p['blacklist_duration_s'] if retry_delay is None else retry_delay
        self.blacklist.append((candidate.x, candidate.y, time.monotonic()+duration))
        self.event('candidate_failed', goal=[candidate.x, candidate.y], reason=reason)

    def finish(self, op, result, error):
        if self.operation is not op:
            return
        self.operation = None
        candidate, kind = op['candidate'], op['kind']
        self.phase, self.reason = 'waiting', ''
        self.next_selection_wall = time.monotonic()+.5
        success = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        if op['cancel']:
            self.event('action_terminated', kind=kind, reason=op['cancel'],
                       status=result.status if result else None)
            if candidate and op['cancel'] in ('goal_timeout', 'no_motion_progress', 'unsafe_goal'):
                self.reject_candidate(candidate, op['cancel'])
            if self.terminal:
                self.phase, self.reason = self.terminal, self.terminal
            return
        if not self.enabled:
            self.phase, self.reason = self.terminal or 'stopped', self.terminal or 'stopped'
            return
        if kind == 'spin':
            self.bootstrap_done = success
            if not success:
                self.bootstrap_failures += 1
            self.last_growth_wall = time.monotonic()
            self.event('initial_observation', succeeded=success, error=error)
            self.next_selection_wall = time.monotonic()+max(3., self.p['settle_time_s'])
            if self.bootstrap_failures >= 3:
                self.stop('initial_observation_failed')
            return
        if kind == 'plan':
            code = getattr(result.result, 'error_code', -1) if result else -1
            # Do not blacklist candidate geometry for infrastructure/start failures.
            if not success and code not in (204, 206, 208):
                self.reason = 'planner_unavailable: '+str(code)
                self.next_selection_wall = time.monotonic()+3.
                self.event('planner_retry', error=error, error_code=code)
                return
            points = [(p.pose.position.x, p.pose.position.y) for p in result.result.path.poses] if success else []
            frame = result.result.path.header.frame_id if success else ''
            safe = safe_cells(self.grid, self.p['robot_radius'], self.p['free_threshold'])
            if not success or frame != self.p['map_frame'] or not path_is_safe(self.grid, safe, points):
                self.reject_candidate(candidate, error or 'unreachable_or_unsafe_path: '+str(code))
                return
            if self.healthy():
                return
            goal = NavigateToPose.Goal(pose=self.pose_message(candidate))
            self.event('goal_selected', **asdict(candidate), path_length_m=sum(
                math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:])))
            self.send('navigate', goal, candidate)
            return
        if success:
            self.successes += 1
            self.blacklist.append((candidate.x, candidate.y, time.monotonic()+self.p['revisit_delay_s']))
            self.event('goal_succeeded', goal=[candidate.x, candidate.y], known_cells=self.known_cells)
            self.next_selection_wall = time.monotonic()+self.p['settle_time_s']
        else:
            self.reject_candidate(candidate, error or getattr(result.result, 'error_msg', 'navigation_failed'))

    def nav2_ready(self):
        now = time.monotonic()
        if now-self.last_lifecycle_query >= 2.:
            self.last_lifecycle_query = now
            for name, client in self.lifecycle_clients.items():
                if name in self.lifecycle_pending:
                    continue
                if not client.service_is_ready():
                    self.lifecycle_states[name] = 0
                    continue
                future = client.call_async(GetState.Request())
                self.lifecycle_pending[name] = future
                future.add_done_callback(lambda f, n=name: self.lifecycle_reply(n, f))
        return (all(self.lifecycle_states.get(name) == 3 for name in self.lifecycle_clients)
                and all(self.nav_clients[k].server_is_ready() for k in ('plan', 'navigate')))

    def lifecycle_reply(self, name, future):
        self.lifecycle_pending.pop(name, None)
        try:
            self.lifecycle_states[name] = future.result().current_state.id
        except Exception:
            self.lifecycle_states[name] = 0

    def tick(self):
        now = time.monotonic()
        self.blacklist = [p for p in self.blacklist if p[2] > now]
        if not self.enabled:
            self.publish()
            return
        if self.p['max_duration_s'] > 0 and now-self.enabled_wall > self.p['max_duration_s']:
            self.stop('duration_limit')
            return
        problem = self.healthy()
        if problem:
            if self.operation is not None:
                self.cancel(problem)
            else:
                self.phase, self.reason = 'paused', problem
            self.publish()
            return
        if self.operation is not None:
            op = self.operation
            if not op['cancel']:
                limit = self.p['plan_timeout_s'] if op['kind'] == 'plan' else self.p['goal_timeout_s']
                if now-op['started'] > limit:
                    self.cancel('goal_timeout')
                elif op['kind'] == 'navigate':
                    pose, old = self.pose(), op['progress_pose']
                    if pose and old and (math.hypot(pose[0]-old[0], pose[1]-old[1]) > .1 or
                                         abs(math.atan2(math.sin(pose[2]-old[2]), math.cos(pose[2]-old[2]))) > .15):
                        op['progress_wall'], op['progress_pose'] = now, pose
                    if now-op['progress_wall'] > self.p['progress_timeout_s']:
                        self.cancel('no_motion_progress')
                    if now-self.last_analysis_wall >= self.p['selection_interval_s']:
                        self.last_analysis_wall = now
                        safe = safe_cells(self.grid, self.p['robot_radius'], self.p['free_threshold'])
                        c, r = self.grid.cell(op['candidate'].x, op['candidate'].y)
                        if not (0 <= r < safe.shape[0] and 0 <= c < safe.shape[1] and safe[r, c]):
                            self.cancel('unsafe_goal')
            self.publish()
            return
        if not self.nav2_ready():
            self.phase, self.reason = 'waiting', 'waiting_for_nav2'
        elif not self.bootstrap_done:
            if now >= self.next_selection_wall and self.nav_clients['spin'].server_is_ready():
                goal = Spin.Goal(target_yaw=self.p['initial_spin_angle'])
                goal.time_allowance.sec = 30
                self.send('spin', goal)
        elif now >= max(self.next_selection_wall, self.last_analysis_wall+self.p['selection_interval_s']):
            self.last_analysis_wall = now
            self.analysis = extract(self.grid, self.pose()[:2], robot_radius=self.p['robot_radius'],
                                    free_threshold=self.p['free_threshold'],
                                    min_frontier_length=self.p['min_frontier_length'],
                                    approach_distance=self.p['approach_distance'],
                                    min_goal_distance=self.p['min_goal_distance'],
                                    goal_clearance_margin=self.p['goal_clearance_margin'],
                                    max_candidates=self.p['max_candidates'])
            choices = [c for c in self.analysis.candidates if all(
                math.hypot(c.x-x, c.y-y) >= self.p['blacklist_radius'] for x, y, _ in self.blacklist)]
            self.show_frontiers(choices)
            if choices:
                self.empty_rounds = 0
                goal = ComputePathToPose.Goal(goal=self.pose_message(choices[0]), use_start=False)
                self.send('plan', goal, choices[0])
            elif self.analysis.reason == 'robot_outside_safe_space':
                self.phase, self.reason = 'paused', self.analysis.reason
            elif self.analysis.candidates:
                self.phase, self.reason = 'waiting', 'candidates_temporarily_blacklisted'
            else:
                self.empty_rounds += 1
                self.phase, self.reason = 'waiting', self.analysis.reason
                if self.empty_rounds >= self.p['stable_empty_rounds']:
                    # A sparse map or narrow passage can hide valid goals.
                    # Do not report completed exploration while eligible
                    # frontiers remain but cannot be safely approached.
                    self.stop('exhausted' if self.analysis.reason ==
                              'no_eligible_frontiers' else 'blocked_frontiers')
            if self.enabled and now-self.last_growth_wall > self.p['stagnation_timeout_s']:
                self.stop('stalled')
        self.publish()

    def show_frontiers(self, candidates):
        array = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        array.markers.append(clear)
        for i, candidate in enumerate(candidates):
            marker = Marker()
            marker.header.frame_id = self.p['map_frame']
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type, marker.action = 'safe_frontier_goals', i, Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = candidate.x, candidate.y, .15
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = .20
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = .15, .85, .35, .9
            array.markers.append(marker)
        self.marker_pub.publish(array)

    def publish(self, force=False):
        now = time.monotonic()
        if not force and now-self.last_publish_wall < 1.:
            return
        self.last_publish_wall = now
        result = {'schema_version': 1, 'phase': self.phase, 'reason': self.reason,
                  'enabled': self.enabled, 'terminal': self.terminal,
                  'wall_elapsed_s': round(now-self.start_wall, 3),
                  'initial_known_cells': self.initial_known_cells, 'known_cells': self.known_cells,
                  'known_gain_cells': self.known_cells-self.initial_known_cells,
                  'known_area_m2': self.known_cells*self.grid.resolution**2 if self.grid else 0.,
                  'successful_goals': self.successes, 'failed_candidates': self.failures,
                  'map_resets': self.map_resets, 'blacklisted_goals': len(self.blacklist),
                  'frontier_cells': self.analysis.frontier_cells if self.analysis else 0,
                  'frontier_clusters': self.analysis.frontier_clusters if self.analysis else 0,
                  'candidate_goals': len(self.analysis.candidates) if self.analysis else 0,
                  'scan_age_s': round(now-self.last_scan_wall, 3),
                  'map_age_s': round(now-self.last_map_wall, 3),
                  'active_goal': asdict(self.operation['candidate']) if self.operation and self.operation['candidate'] else None,
                  'history': self.history}
        text = json.dumps(result, ensure_ascii=False, allow_nan=False)
        self.status_pub.publish(String(data=text))
        if self.p['report_file']:
            path = Path(self.p['report_file']).expanduser()
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                tmp = path.with_name(path.name+'.tmp')
                tmp.write_text(text+'\n', encoding='utf-8')
                tmp.replace(path)
            except OSError as exc:
                self.get_logger().error('Cannot save exploration report: '+str(exc))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = FrontierExplorer()
    interrupted = False

    def interrupt(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        while rclpy.ok() and not interrupted:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.stop('shutdown')
        deadline = time.monotonic()+5.
        while rclpy.ok() and node.operation is not None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if node.operation is not None:
            node.phase, node.reason = 'cancel_unconfirmed', 'Nav2 did not confirm termination before shutdown'
        node.publish(force=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
