#!/usr/bin/env python3
"""ROS action lifecycle regressions, without a robot or an action server."""
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

import numpy as np
import rclpy
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose, ComputePathToPose, Spin
from std_srvs.srv import SetBool

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from frontier_core import Candidate, Grid
from frontier_explorer import FrontierExplorer


class FakeHandle:
    accepted = True

    def __init__(self):
        self.result = Future()
        self.cancels = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancels += 1
        done = Future()
        done.set_result(SimpleNamespace(return_code=0))
        return done


class FakeClient:
    def __init__(self):
        self.sends = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        reply = Future()
        self.sends.append((goal, reply))
        return reply


class LifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = FrontierExplorer()
        self.original_clients = list(self.node.nav_clients.values())
        self.node.nav_clients = {kind: FakeClient() for kind in self.node.nav_clients}
        self.node.grid = Grid(np.zeros((60, 60), np.int16), .1, -3., -3.)
        self.node.pose = lambda: (0., 0., 0.)
        self.node.healthy = lambda: ''
        self.node.last_growth_wall = time.monotonic()
        self.candidate = Candidate(1., 0., 0., 1.5, 0., 1., .5)

    def tearDown(self):
        for client in self.original_clients:
            client.destroy()
        self.node.destroy_node()

    def test_action_server_presence_requires_active_nav2_nodes(self):
        # Discovery of actions alone must not start observation during bringup.
        self.node.last_lifecycle_query = time.monotonic()
        self.assertFalse(self.node.nav2_ready())
        self.node.lifecycle_states = {name: 3 for name in self.node.lifecycle_clients}
        self.assertTrue(self.node.nav2_ready())
        self.node.lifecycle_states['behavior_server'] = 2
        self.assertFalse(self.node.nav2_ready())

    def test_sparse_seed_cannot_be_reported_as_exhausted(self):
        cells = np.full((60, 60), -1, np.int16)
        rr, cc = np.ogrid[:60, :60]
        cells[(rr-30)**2+(cc-30)**2 <= 7**2] = 0
        self.node.grid = Grid(cells, .1, -3.05, -3.05)
        self.node.bootstrap_done = True
        self.node.nav2_ready = lambda: True
        for _ in range(self.node.p['stable_empty_rounds']):
            self.node.last_analysis_wall = 0.
            self.node.tick()
        self.assertGreater(self.node.analysis.frontier_clusters, 0)
        self.assertEqual(self.node.nav_clients['navigate'].sends, [])
        self.assertEqual(self.node.terminal, 'blocked_frontiers')

    def test_rejected_initial_observation_is_not_completion(self):
        self.node.send('spin', Spin.Goal())
        op = self.node.operation
        self.node.finish(op, None, 'goal_rejected')
        self.assertFalse(self.node.bootstrap_done)
        self.assertTrue(self.node.enabled)
        self.assertEqual(self.node.terminal, '')
        self.assertGreater(self.node.next_selection_wall, time.monotonic())

    def test_stop_before_acceptance_cancels_late_goal(self):
        self.node.send('navigate', NavigateToPose.Goal(), self.candidate)
        self.node.stop('stopped')
        handle = FakeHandle()
        self.node.nav_clients['navigate'].sends[0][1].set_result(handle)
        self.assertEqual(handle.cancels, 1)
        self.assertIsNotNone(self.node.operation)
        response = self.node.on_enable(SetBool.Request(data=True), SetBool.Response())
        self.assertFalse(response.success)
        handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED, result=NavigateToPose.Result()))
        self.assertIsNone(self.node.operation)
        self.assertEqual(self.node.phase, 'stopped')
        self.assertEqual(self.node.successes, 0)

    def test_cancel_ack_cannot_start_successor(self):
        self.node.send('navigate', NavigateToPose.Goal(), self.candidate)
        handle = FakeHandle()
        self.node.nav_clients['navigate'].sends[0][1].set_result(handle)
        self.node.cancel('goal_timeout')
        self.node.tick()
        self.assertEqual(self.node.phase, 'canceling')
        self.assertEqual(self.node.nav_clients['plan'].sends, [])
        # A SUCCESS racing a cancellation is still not permission to chain goals.
        handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=NavigateToPose.Result()))
        self.assertEqual(self.node.successes, 0)
        self.assertEqual(self.node.failures, 1)

    def test_planner_tf_failure_does_not_blacklist_geometry(self):
        self.node.send('plan', ComputePathToPose.Goal(), self.candidate)
        op = self.node.operation
        result = ComputePathToPose.Result(error_code=ComputePathToPose.Result.TF_ERROR)
        self.node.finish(op, SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=result), '')
        self.assertEqual(self.node.blacklist, [])
        self.assertEqual(self.node.failures, 0)
        self.assertIn('planner_unavailable', self.node.reason)

    def test_no_valid_path_is_blacklisted(self):
        self.node.send('plan', ComputePathToPose.Goal(), self.candidate)
        op = self.node.operation
        result = ComputePathToPose.Result(error_code=ComputePathToPose.Result.NO_VALID_PATH)
        self.node.finish(op, SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=result), '')
        self.assertEqual(self.node.failures, 1)
        self.assertEqual(len(self.node.blacklist), 1)
        self.assertEqual(self.node.nav_clients['navigate'].sends, [])

    def test_sensor_failure_requests_cancel_instead_of_new_target(self):
        self.node.send('navigate', NavigateToPose.Goal(), self.candidate)
        handle = FakeHandle()
        self.node.nav_clients['navigate'].sends[0][1].set_result(handle)
        self.node.healthy = lambda: 'scan_invalid_or_stale'
        self.node.tick()
        self.assertEqual(handle.cancels, 1)
        self.assertEqual(self.node.nav_clients['plan'].sends, [])


if __name__ == '__main__':
    unittest.main()
