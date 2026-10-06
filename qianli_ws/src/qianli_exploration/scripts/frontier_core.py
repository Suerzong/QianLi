#!/usr/bin/env python3
"""ROS-independent frontier extraction; coordinates are OccupancyGrid coordinates.

Unknown cells are never treated as traversable. A circumscribed, padded robot
circle deliberately sacrifices some tight passages to keep approach goals and
paths safe for every heading. Nav2 remains the final path/obstacle authority.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

cv2.setNumThreads(1)


@dataclass(frozen=True)
class Grid:
    cells: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def world(self, col, row):
        x, y = (col + .5) * self.resolution, (row + .5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return self.origin_x + c*x - s*y, self.origin_y + s*x + c*y

    def cell(self, x, y):
        dx, dy = x-self.origin_x, y-self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return math.floor((c*dx+s*dy)/self.resolution), math.floor((-s*dx+c*dy)/self.resolution)


@dataclass(frozen=True)
class Candidate:
    x: float
    y: float
    yaw: float
    frontier_x: float
    frontier_y: float
    gain_m2: float
    score: float


@dataclass
class Analysis:
    candidates: list
    safe: np.ndarray
    frontier_cells: int
    frontier_clusters: int
    reason: str


def safe_cells(grid, robot_radius, free_threshold=20):
    free = (grid.cells >= 0) & (grid.cells <= free_threshold)
    # Zero padding makes outside-map space an obstacle, even in all-free maps.
    distance = cv2.distanceTransform(np.pad(free.astype(np.uint8), 1),
                                     cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    # Distances are between cell centres; reserve the obstacle cell half-diagonal.
    return free & (distance*grid.resolution >= robot_radius + grid.resolution/math.sqrt(2))


def extract(grid, robot_xy, *, robot_radius=.47, free_threshold=20,
            min_frontier_length=.35, approach_distance=1.5,
            min_goal_distance=.55, goal_clearance_margin=.1, sample_spacing=1.5, gain_radius=2.0,
            max_candidates=16, excluded=(), exclusion_radius=.65):
    if grid.cells.ndim != 2 or grid.resolution <= 0 or not np.isfinite(grid.resolution):
        raise ValueError('Expected a finite, positive-resolution 2D grid')
    free = (grid.cells >= 0) & (grid.cells <= free_threshold)
    unknown = np.pad(grid.cells < 0, 1, constant_values=True)
    boundary = free & (unknown[:-2, 1:-1] | unknown[2:, 1:-1] |
                       unknown[1:-1, :-2] | unknown[1:-1, 2:])
    safe = safe_cells(grid, robot_radius, free_threshold)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(boundary.astype(np.uint8), connectivity=8)
    clusters = [i for i in range(1, count)
                if stats[i, cv2.CC_STAT_AREA]*grid.resolution >= min_frontier_length]
    frontier_count = int(boundary.sum())
    if not clusters:
        return Analysis([], safe, frontier_count, 0, 'no_eligible_frontiers')
    col, row = grid.cell(*robot_xy)
    h, w = safe.shape
    if not (0 <= row < h and 0 <= col < w and safe[row, col]):
        return Analysis([], safe, frontier_count, len(clusters), 'robot_outside_safe_space')
    _, regions = cv2.connectedComponents(safe.astype(np.uint8), connectivity=4)
    reachable = regions == regions[row, col]
    # Extra room at the observation goal prevents the final approach
    # from clipping unknown corners. Keep corridor connectivity at
    # the body radius, so this goal margin does not seal narrow doors.
    goal_cells = reachable & safe_cells(grid, robot_radius+goal_clearance_margin, free_threshold)
    approach_px = math.ceil(approach_distance/grid.resolution)
    gain_px = math.ceil(gain_radius/grid.resolution)
    spacing_px = max(1, round(sample_spacing/grid.resolution))
    proposals = []
    for label in clusters:
        rows, cols = np.nonzero(labels == label)
        bins = (rows//spacing_px)*(math.ceil(w/spacing_px)+1) + cols//spacing_px
        for bin_id in np.unique(bins):
            rr, cc = rows[bins == bin_id], cols[bins == bin_id]
            centre_r, centre_c = rr.mean(), cc.mean()
            nearest = np.argmin((rr-centre_r)**2+(cc-centre_c)**2)
            fr, fc = int(rr[nearest]), int(cc[nearest])
            r0, r1 = max(0, fr-approach_px), min(h, fr+approach_px+1)
            c0, c1 = max(0, fc-approach_px), min(w, fc+approach_px+1)
            sr, sc = np.nonzero(goal_cells[r0:r1, c0:c1])
            sr, sc = sr+r0, sc+c0
            if len(sr) == 0:
                continue
            distances = (sr-fr)**2+(sc-fc)**2
            valid = distances*grid.resolution**2 <= approach_distance**2
            robot_distance = np.hypot(sr-row, sc-col)*grid.resolution
            valid &= robot_distance >= min_goal_distance
            sr, sc, distances = sr[valid], sc[valid], distances[valid]
            if len(sr) == 0:
                continue
            # Pick a safe interior point close to this boundary, not its centroid.
            best = int(np.argmin(distances))
            x, y = grid.world(int(sc[best]), int(sr[best]))
            if any(math.hypot(x-ex, y-ey) < exclusion_radius for ex, ey in excluded):
                continue
            fx, fy = grid.world(fc, fr)
            gr0, gr1 = max(0, fr-gain_px), min(h, fr+gain_px+1)
            gc0, gc1 = max(0, fc-gain_px), min(w, fc+gain_px+1)
            yr, xc = np.ogrid[gr0:gr1, gc0:gc1]
            disk = (yr-fr)**2+(xc-fc)**2 <= gain_px**2
            gain = float(((grid.cells[gr0:gr1, gc0:gc1] < 0) & disk).sum())*grid.resolution**2
            # Map-edge frontiers can reveal space outside today's map bounds.
            if fr in (0, h-1) or fc in (0, w-1):
                gain = max(gain, gain_radius**2)
            score = gain/(1.0+math.hypot(x-robot_xy[0], y-robot_xy[1]))
            proposals.append(Candidate(x, y, math.atan2(fy-y, fx-x), fx, fy, gain, score))
    proposals.sort(key=lambda p: (-p.score, p.x, p.y))
    selected = []
    for candidate in proposals:
        if all(math.hypot(candidate.x-p.x, candidate.y-p.y) >= .6 for p in selected):
            selected.append(candidate)
        if len(selected) >= max_candidates:
            break
    return Analysis(selected, safe, frontier_count, len(clusters),
                    'candidates' if selected else 'no_reachable_frontiers')


def path_is_safe(grid, safe, points):
    """Check whole segments, not only Nav2's sampled path vertices."""
    if not points:
        return False
    h, w = safe.shape
    for a, b in zip(points, points[1:] or points):
        steps = max(1, math.ceil(math.hypot(b[0]-a[0], b[1]-a[1])/(grid.resolution*.5)))
        for t in np.linspace(0, 1, steps+1):
            col, row = grid.cell(a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
            if not (0 <= col < w and 0 <= row < h and safe[row, col]):
                return False
    return True
