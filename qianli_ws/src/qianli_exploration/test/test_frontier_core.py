#!/usr/bin/env python3
"""Behavioral tests: reachability, body clearance, map transforms and paths."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from frontier_core import Grid, extract, path_is_safe, safe_cells, travel_distances, visible_unknown_gain, segment_is_known_free, remaining_path


class FrontierTests(unittest.TestCase):
    def grid(self, cells, yaw=0):
        return Grid(cells, .1, -2.5, -2.5, yaw)

    def test_gain_does_not_see_unknown_through_a_known_wall(self):
        cells = np.full((80, 80), -1, np.int16)
        cells[:, :40] = 0
        cells[:, 40] = 100
        grid = self.grid(cells)
        self.assertEqual(visible_unknown_gain(grid, 30, 40, 2.), 0.)
        cells[35:46, 40] = 0
        self.assertGreater(visible_unknown_gain(grid, 30, 40, 2.), 0.)

    def test_visible_map_edge_can_gain_without_truth_map(self):
        grid = self.grid(np.zeros((80, 80), np.int16))
        self.assertGreater(visible_unknown_gain(grid, 70, 40, 2.), 0.)
        self.assertEqual(visible_unknown_gain(grid, 40, 40, 2.), 0.)

    def test_travel_cost_accounts_for_wall_detour(self):
        reachable = np.ones((11, 11), bool)
        reachable[:9, 5] = False
        distances = travel_distances(reachable, (4, 1), 1.)
        # Across a one-cell wall is 2 m Euclidean, but 18 m via its opening.
        self.assertEqual(distances[1, 6], 18.)
        self.assertEqual(distances[5, 4], 4.)
        self.assertTrue(np.isinf(distances[1, 5]))

    def test_safe_subcell_pose_is_not_rejected_by_cell_centre(self):
        cells = np.zeros((30,30),np.int16);cells[:,5]=100;cells[:,20:]=-1
        grid = self.grid(cells);pose=(-1.42,-1.0)
        c,r = grid.cell(*pose)
        self.assertFalse(safe_cells(grid,.47)[r,c])
        self.assertTrue(segment_is_known_free(grid,pose,pose,.47))
        self.assertTrue(extract(grid,pose).candidates)
        self.assertFalse(segment_is_known_free(grid,(-1.44,-1.),(-1.44,-1.),.47))

    def test_swept_circle_cannot_clip_a_rectangle_corner(self):
        cells = np.zeros((30,30),np.int16);cells[10,10]=100
        grid = self.grid(cells)
        self.assertFalse(segment_is_known_free(grid,(-1.8,-1.55),(-1.1,-1.55),.1))
        self.assertTrue(segment_is_known_free(grid,(-1.8,-1.61),(-1.1,-1.61),.1))

    def test_continuous_segment_intersection_ignores_vertex_spacing(self):
        cells = np.zeros((30,30),np.int16);cells[10,10]=-1
        grid = self.grid(cells)
        self.assertFalse(segment_is_known_free(grid,(-1.8,-1.8),(-1.1,-1.1),.01))

    def test_rotated_map_swept_circle_geometry(self):
        cells = np.zeros((30,30),np.int16);cells[10,10]=100
        grid = self.grid(cells,.7)
        self.assertFalse(segment_is_known_free(grid,grid.world(5,10),grid.world(15,10),.05))
        self.assertTrue(segment_is_known_free(grid,grid.world(5,5),grid.world(15,5),.05))

    def test_remaining_path_excludes_passed_segments(self):
        result = remaining_path([(0.,0.),(0.,1.),(1.,1.)],(.9,1.))
        self.assertEqual(result[0],(.9,1.))
        self.assertEqual(result[-1],(1.,1.))
        self.assertNotIn((0.,0.),result)
        self.assertNotIn((0.,1.),result)

    def test_unknown_is_never_a_target(self):
        cells = np.full((50, 50), -1, np.int16)
        cells[5:45, 5:25] = 0
        grid = self.grid(cells)
        analysis = extract(grid, grid.world(12, 25))
        self.assertTrue(analysis.candidates)
        for candidate in analysis.candidates:
            c, r = grid.cell(candidate.x, candidate.y)
            self.assertEqual(cells[r, c], 0)
            self.assertTrue(analysis.safe[r, c])
            self.assertLessEqual(math.hypot(candidate.x-candidate.frontier_x,
                                           candidate.y-candidate.frontier_y), 1.5)

    def test_goal_margin_keeps_body_reachable_door_open(self):
        cells = np.full((70, 100), 100, np.int16)
        cells[10:60, 5:35] = 0
        cells[10:60, 45:75] = 0
        cells[30:41, 35:45] = 0  # Body fits; the extra observation margin does not.
        cells[10:60, 75:95] = -1
        grid = self.grid(cells)
        result = extract(grid, grid.world(20, 35), goal_clearance_margin=.1)
        self.assertTrue(result.candidates)
        deep = safe_cells(grid, .57)
        for candidate in result.candidates:
            c, r = grid.cell(candidate.x, candidate.y)
            self.assertGreater(c, 45)
            self.assertTrue(deep[r, c])
        self.assertTrue(result.safe[35, 40])
        self.assertFalse(deep[35, 40])

    def test_disconnected_frontier_is_excluded(self):
        cells = np.full((80, 100), 100, np.int16)
        cells[10:70, 5:35] = 0
        cells[10:70, 65:85] = 0
        cells[10:70, 85:95] = -1
        grid = self.grid(cells)
        result = extract(grid, grid.world(15, 35))
        self.assertGreater(result.frontier_clusters, 0)
        self.assertEqual(result.candidates, [])
        self.assertEqual(result.reason, 'no_reachable_frontiers')

    def test_narrow_door_is_not_used_to_reach_far_frontier(self):
        cells = np.full((90, 90), 100, np.int16)
        cells[10:80, 5:40] = 0
        cells[10:80, 50:75] = 0
        cells[43:48, 40:50] = 0  # 0.5 m doorway, narrower than the robot.
        cells[10:80, 75:85] = -1
        grid = self.grid(cells)
        self.assertEqual(extract(grid, grid.world(20, 45)).candidates, [])

    def test_closed_known_room_finishes_without_crossing_wall(self):
        cells = np.full((50, 50), -1, np.int16)
        cells[5:45, 5:45] = 100
        cells[6:44, 6:44] = 0
        result = extract(self.grid(cells), (0., 0.))
        self.assertEqual(result.frontier_cells, 0)
        self.assertEqual(result.reason, 'no_eligible_frontiers')

    def test_rotated_origin_and_resize_keep_world_coordinates(self):
        cells = np.zeros((50, 50), np.int16)
        grid = self.grid(cells, math.pi/2)
        point = grid.world(12, 23)
        self.assertEqual(grid.cell(*point), (12, 23))
        # Extend left and below: the same physical cell shifts by 5 columns/rows.
        dx, dy = -.5, -.5
        resized = Grid(np.zeros((60, 60)), .1,
                       grid.origin_x-dy, grid.origin_y+dx, math.pi/2)
        self.assertEqual(resized.cell(*point), (17, 28))
        self.assertAlmostEqual(resized.world(17, 28)[0], point[0])
        self.assertAlmostEqual(resized.world(17, 28)[1], point[1])

    def test_sparse_path_vertices_cannot_jump_over_wall(self):
        cells = np.zeros((60, 60), np.int16)
        cells[:, 30] = 100
        grid = self.grid(cells)
        safe = safe_cells(grid, .47)
        self.assertTrue(safe[25, 10] and safe[25, 45])
        self.assertFalse(path_is_safe(grid, safe, [grid.world(10, 25), grid.world(45, 25)]))
        self.assertTrue(path_is_safe(grid, safe, [grid.world(10, 15), grid.world(10, 35)]))

    def test_blacklist_does_not_follow_map_array_indices(self):
        cells = np.full((50, 50), -1, np.int16)
        cells[5:45, 5:25] = 0
        grid = self.grid(cells)
        first = extract(grid, grid.world(12, 25))
        self.assertTrue(first.candidates)
        exclusions = [(p.x, p.y) for p in first.candidates]
        result = extract(grid, grid.world(12, 25), excluded=exclusions)
        for candidate in result.candidates:
            self.assertTrue(all(math.hypot(candidate.x-x, candidate.y-y) >= .65 for x, y in exclusions))

    def test_exclusion_before_top_k_keeps_farther_goals_available(self):
        cells = np.full((70, 70), -1, np.int16)
        cells[5:65, 5:40] = 0
        grid = self.grid(cells)
        pose = grid.world(20, 30)
        first = extract(grid, pose, max_candidates=1).candidates[0]
        result = extract(grid, pose, max_candidates=1, excluded=[(first.x, first.y)])
        self.assertEqual(len(result.candidates), 1)
        self.assertGreaterEqual(math.hypot(result.candidates[0].x-first.x,
                                          result.candidates[0].y-first.y), .65)

    def test_all_excluded_is_waiting_not_blocked_or_exhausted(self):
        cells = np.full((50, 50), -1, np.int16)
        cells[5:45, 5:25] = 0
        grid = self.grid(cells)
        result = extract(grid, grid.world(12, 25), excluded=[(0., 0.)], exclusion_radius=100.)
        self.assertEqual(result.candidates, [])
        self.assertEqual(result.reason, 'candidates_temporarily_blacklisted')

    def test_robot_at_unknown_edge_does_not_produce_false_completion(self):
        cells = np.full((50, 50), -1, np.int16)
        cells[5:45, 5:25] = 0
        grid = self.grid(cells)
        result = extract(grid, grid.world(24, 25))
        self.assertEqual(result.reason, 'robot_outside_safe_space')


if __name__ == '__main__':
    unittest.main()
