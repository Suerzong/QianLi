#!/usr/bin/env python3
"""Check manifest tasks with Nav2 ComputePathToPose without moving the robot.

Each query supplies its own explicit start (use_start=True). An expected
unreachable goal passes only for STATUS_ABORTED together with the actual
ComputePathToPose.Result.NO_VALID_PATH constant. GOAL_OCCUPIED is recorded as
different failure evidence and never counted as the desired disconnection.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import traceback


def write_report(filename, report):
    path = Path(filename)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n', encoding='utf-8')
    os.replace(temporary, path)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def check_pose(pose, name):
    if len(pose) != 3 or not all(math.isfinite(float(value)) for value in pose):
        raise ValueError('{} must be a finite [x, y, yaw] pose'.format(name))
    return [float(value) for value in pose]


def create_node(options, report):
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    result_type = ComputePathToPose.Result
    if not hasattr(result_type, 'NO_VALID_PATH'):
        raise RuntimeError('Installed ComputePathToPose.Result lacks NO_VALID_PATH; refusing to guess its numeric value')
    constants = {name: int(getattr(result_type, name))
                 for name in dir(result_type)
                 if name.isupper() and isinstance(getattr(result_type, name), int)}
    statuses = {name: int(getattr(GoalStatus, name)) for name in dir(GoalStatus)
                if name.startswith('STATUS_') and isinstance(getattr(GoalStatus, name), int)}
    report['result_constants'] = constants
    report['goal_status_constants'] = statuses
    no_valid_path = result_type.NO_VALID_PATH
    goal_occupied = getattr(result_type, 'GOAL_OCCUPIED', None)
    status_names = {value: name for name, value in statuses.items()}
    error_names = {}
    for name, value in constants.items():
        error_names.setdefault(value, []).append(name)

    class PlanSuite(Node):
        def __init__(self):
            super().__init__('qianli_teaching_building_plan_suite',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.client = ActionClient(self, ComputePathToPose, options.action)
            self.active_handle, self.pending_send = None, None
            self.last_result = None

        def cancel_late_acceptance(self, future):
            try:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                pass

        def cancel(self):
            if self.pending_send is not None:
                pending = self.pending_send
                deadline = time.monotonic() + 2.
                while rclpy.ok() and not pending.done() and time.monotonic() < deadline:
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
                request = self.active_handle.cancel_goal_async()
                deadline = time.monotonic() + 2.
                while rclpy.ok() and not request.done() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=.05)
                self.active_handle = None

        def pose_message(self, target, pose):
            target.header.frame_id = options.frame
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x, target.pose.position.y = pose[0], pose[1]
            target.pose.orientation.z = math.sin(pose[2] / 2)
            target.pose.orientation.w = math.cos(pose[2] / 2)

        def query(self, task):
            start = check_pose(task['start'], task['id'] + ' start')
            goal_pose = check_pose(task['goal'], task['id'] + ' goal')
            expected = task.get('expected', 'reachable')
            if expected not in ('reachable', 'unreachable'):
                raise ValueError('Unknown expected result: {}'.format(expected))
            result = {'id': str(task['id']), 'start': start, 'goal': goal_pose,
                      'expected': expected, 'planner_id': options.planner,
                      'use_start': True, 'accepted': False, 'action_status': None,
                      'action_status_name': None, 'terminal_reason': None,
                      'error_code': None, 'error_names': [], 'error_msg': None,
                      'planning_time_s': None, 'path_length_m': 0., 'path_poses': [],
                      'path_frame': None, 'passed': False, 'evidence_type': None}
            self.last_result = result
            goal = ComputePathToPose.Goal()
            goal.planner_id, goal.use_start = options.planner, True
            self.pose_message(goal.start, start)
            self.pose_message(goal.goal, goal_pose)
            begun = time.monotonic()
            try:
                sent = self.client.send_goal_async(goal)
                self.pending_send = sent
                while rclpy.ok() and not sent.done():
                    if time.monotonic() - begun >= options.timeout:
                        result['terminal_reason'] = 'goal_acceptance_timeout'
                        self.cancel()
                        return result
                    rclpy.spin_once(self, timeout_sec=.05)
                if not rclpy.ok():
                    result['terminal_reason'] = 'ros_context_stopped'
                    return result
                handle = sent.result()
                self.pending_send = None
                if not handle.accepted:
                    result['terminal_reason'] = 'goal_rejected'
                    result['evidence_type'] = 'action_rejected_without_planner_result'
                    return result
                self.active_handle = handle
                result['accepted'] = True
                complete = handle.get_result_async()
                while rclpy.ok() and not complete.done():
                    if time.monotonic() - begun >= options.timeout:
                        result['terminal_reason'] = 'wall_timeout'
                        self.cancel()
                        deadline = time.monotonic() + 2.
                        while rclpy.ok() and not complete.done() and time.monotonic() < deadline:
                            rclpy.spin_once(self, timeout_sec=.05)
                        break
                    rclpy.spin_once(self, timeout_sec=.05)
                if not complete.done():
                    result['terminal_reason'] = result['terminal_reason'] or 'ros_context_stopped'
                    return result
                wrapped = complete.result()
                planner_result = wrapped.result
                result['action_status'] = int(wrapped.status)
                result['action_status_name'] = status_names.get(int(wrapped.status), 'UNKNOWN')
                result['error_code'] = int(planner_result.error_code)
                result['error_names'] = error_names.get(result['error_code'], [])
                result['error_msg'] = planner_result.error_msg
                result['planning_time_s'] = planner_result.planning_time.sec + planner_result.planning_time.nanosec * 1e-9
                result['path_frame'] = planner_result.path.header.frame_id
                result['path_poses'] = [[p.pose.position.x, p.pose.position.y,
                                         yaw(p.pose.orientation)]
                                        for p in planner_result.path.poses]
                poses = result['path_poses']
                result['path_length_m'] = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                              for a, b in zip(poses, poses[1:]))
                timed_out = result['terminal_reason'] == 'wall_timeout'
                result['terminal_reason'] = result['terminal_reason'] or result['action_status_name'].lower()
                succeeded = wrapped.status == GoalStatus.STATUS_SUCCEEDED and bool(poses)
                disconnected = (wrapped.status == GoalStatus.STATUS_ABORTED
                                and planner_result.error_code == no_valid_path)
                occupied = (wrapped.status == GoalStatus.STATUS_ABORTED
                            and goal_occupied is not None
                            and planner_result.error_code == goal_occupied)
                if occupied:
                    result['evidence_type'] = 'goal_occupied'
                    result['failure_evidence'] = {
                        'meaning': 'Goal lies in occupied costmap space; this is not proof of a free but disconnected destination.',
                        'actual_result_constant': 'ComputePathToPose.Result.GOAL_OCCUPIED',
                        'actual_result_value': int(goal_occupied),
                        'action_status': int(wrapped.status), 'error_msg': planner_result.error_msg}
                elif disconnected:
                    result['evidence_type'] = 'no_valid_path'
                elif succeeded:
                    result['evidence_type'] = 'nonempty_planned_path'
                elif wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                    result['evidence_type'] = 'succeeded_with_empty_path'
                else:
                    result['evidence_type'] = 'other_planner_failure'
                result['passed'] = not timed_out and (succeeded if expected == 'reachable' else disconnected)
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
                result['elapsed_wall_s'] = time.monotonic() - begun

    return PlanSuite()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--action', default='/compute_path_to_pose')
    parser.add_argument('--planner', default='GridBased')
    parser.add_argument('--frame', default='map')
    parser.add_argument('--timeout', type=float, default=30., help='Wall seconds per query')
    parser.add_argument('--startup-timeout', type=float, default=60.)
    options = parser.parse_args(argv)
    report = {'schema_version': 1, 'kind': 'explicit_start_planning_suite',
              'manifest': str(Path(options.manifest).resolve()), 'options': vars(options),
              'status': 'starting', 'started_unix_s': time.time(), 'tasks': [],
              'robot_motion_commanded': False,
              'acceptance': {'reachable': 'Actual STATUS_SUCCEEDED with nonempty path.',
                             'unreachable': 'Actual STATUS_ABORTED and ComputePathToPose.Result.NO_VALID_PATH.',
                             'goal_occupied': 'Separate failure evidence; does not pass disconnected-free-goal check.'}}
    write_report(options.output, report)
    ros, node, current_task = None, None, None
    exit_code = 1
    try:
        if options.timeout <= 0 or options.startup_timeout <= 0:
            raise ValueError('Timeout values must be positive')
        manifest = json.loads(Path(options.manifest).read_text(encoding='utf-8'))
        jobs = manifest.get('tasks', [])
        if not jobs:
            raise ValueError('Manifest has no tasks')
        report['requested_tasks'] = len(jobs)
        import rclpy as ros
        from rclpy.signals import SignalHandlerOptions
        ros.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        def interrupted(_signum, _frame):
            raise InterruptedError('Termination signal')
        signal.signal(signal.SIGTERM, interrupted)
        node = create_node(options, report)
        deadline = time.monotonic() + options.startup_timeout
        while ros.ok() and time.monotonic() < deadline:
            if node.client.server_is_ready():
                break
            ros.spin_once(node, timeout_sec=.10)
        else:
            raise TimeoutError('ComputePathToPose server unavailable')
        report['status'] = 'running'
        write_report(options.output, report)
        for task in jobs:
            current_task = {'id': str(task['id']), 'status': 'running', 'passed': False}
            report['tasks'].append(current_task)
            write_report(options.output, report)
            result = node.query(task)
            current_task.update(result)
            current_task['status'] = 'passed' if result['passed'] else 'failed'
            write_report(options.output, report)
            print(json.dumps({key: current_task.get(key) for key in
                              ('id', 'passed', 'action_status_name', 'error_names',
                               'path_length_m', 'planning_time_s', 'evidence_type')},
                             ensure_ascii=False), flush=True)
            if result['terminal_reason'] in ('goal_acceptance_timeout', 'ros_context_stopped'):
                report['status'] = 'stopped_transport_failure'
                break
        else:
            report['status'] = 'completed'
        report['passed_tasks'] = sum(task.get('passed', False) for task in report['tasks'])
        exit_code = 0 if report['status'] == 'completed' and report['passed_tasks'] == len(jobs) else 1
    except (KeyboardInterrupt, InterruptedError) as error:
        report['status'], report['exception'] = 'interrupted', str(error)
        if current_task is not None:
            if node is not None and node.last_result is not None and node.last_result['id'] == current_task['id']:
                current_task.update(node.last_result)
            current_task['status'], current_task['passed'] = 'interrupted', False
        exit_code = 130
    except Exception as error:
        report['status'] = 'failed'
        report['exception'] = '{}: {}'.format(type(error).__name__, error)
        report['traceback'] = traceback.format_exc()
        if current_task is not None:
            current_task['status'], current_task['passed'] = 'failed', False
        print(report['exception'], file=sys.stderr, flush=True)
    finally:
        if node is not None:
            try:
                node.cancel()
                node.destroy_node()
            except Exception as error:
                report['cleanup_exception'] = str(error)
        if ros is not None:
            try:
                if ros.ok():
                    ros.shutdown()
            except Exception as error:
                report['shutdown_exception'] = str(error)
        report['finished_unix_s'], report['exit_code'] = time.time(), exit_code
        write_report(options.output, report)
        print('report', options.output, report['status'], flush=True)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
