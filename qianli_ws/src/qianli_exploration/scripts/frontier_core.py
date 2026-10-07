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
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

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
    anchor: tuple | None = None


def safe_cells(grid, robot_radius, free_threshold=20):
    free = (grid.cells >= 0) & (grid.cells <= free_threshold)
    # Zero padding makes outside-map space an obstacle, even in all-free maps.
    distance = cv2.distanceTransform(np.pad(free.astype(np.uint8), 1),
                                     cv2.DIST_L2, cv2.DIST_MASK_PRECISE)[1:-1, 1:-1]
    # Distances are between cell centres; reserve the obstacle cell half-diagonal.
    return free & (distance*grid.resolution >= robot_radius + grid.resolution/math.sqrt(2))



def segment_is_known_free(grid, a, b, radius, free_threshold=20):
    """Exact swept-circle clearance against occupied/unknown cell rectangles."""
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    def local(point):
        x, y = point[0]-grid.origin_x, point[1]-grid.origin_y
        return np.array([c*x+s*y, -s*x+c*y])
    a, b = local(a), local(b)
    h, w = grid.cells.shape
    low, high = np.minimum(a, b)-radius, np.maximum(a, b)+radius
    if np.any(low <= 0) or high[0] >= w*grid.resolution or high[1] >= h*grid.resolution:
        return False
    c0, r0 = np.floor(low/grid.resolution).astype(int)
    c1, r1 = np.floor(high/grid.resolution).astype(int)+1
    cells = grid.cells[r0:r1, c0:c1]
    rr, cc = np.nonzero((cells < 0) | (cells > free_threshold))
    if not len(rr):return True
    mins = np.column_stack((cc+c0, rr+r0))*grid.resolution
    maxs = mins+grid.resolution
    def endpoint_distance(point):
        delta = np.maximum(np.maximum(mins-point, 0), point-maxs)
        return (delta*delta).sum(axis=1)
    distance = np.minimum(endpoint_distance(a), endpoint_distance(b))
    direction = b-a
    length2 = float(direction@direction)
    if length2:
        # Slab intersection plus all rectangle corners handles every segment/
        # rectangle configuration, including edge interiors and zero slopes.
        t0, t1 = np.zeros(len(rr)), np.ones(len(rr))
        intersects = np.ones(len(rr), bool)
        for axis in (0,1):
            if abs(direction[axis]) < 1e-12:
                intersects &= (a[axis] >= mins[:,axis]) & (a[axis] <= maxs[:,axis])
            else:
                near = (mins[:,axis]-a[axis])/direction[axis]
                far = (maxs[:,axis]-a[axis])/direction[axis]
                t0 = np.maximum(t0, np.minimum(near, far))
                t1 = np.minimum(t1, np.maximum(near, far))
        intersects &= t0 <= t1
        distance[intersects] = 0
        for xside,yside in ((0,0),(0,1),(1,0),(1,1)):
            corner = np.column_stack(((maxs if xside else mins)[:,0],
                                      (maxs if yside else mins)[:,1]))
            t = np.clip(((corner-a)*direction).sum(axis=1)/length2,0,1)
            delta = corner-a-t[:,None]*direction
            distance = np.minimum(distance,(delta*delta).sum(axis=1))
    return bool(np.all(distance > radius*radius+1e-12))


def travel_distances(reachable, start, resolution):
    """Shortest four-connected travel distances through body-safe known cells.

    This route cost is a conservative approximation; Nav2 still computes and
    validates the actual continuous path before any movement.
    """
    ids = np.full(reachable.shape, -1, np.int32)
    ids[reachable] = np.arange(int(reachable.sum()), dtype=np.int32)
    horizontal = reachable[:, :-1] & reachable[:, 1:]
    vertical = reachable[:-1, :] & reachable[1:, :]
    a = np.concatenate((ids[:, :-1][horizontal], ids[:-1, :][vertical]))
    b = np.concatenate((ids[:, 1:][horizontal], ids[1:, :][vertical]))
    n = int(reachable.sum())
    graph = csr_matrix((np.ones(len(a)), (a, b)), shape=(n, n))
    costs = dijkstra(graph, directed=False, indices=int(ids[start[1], start[0]]))
    result = np.full(reachable.shape, np.inf)
    result[reachable] = costs*resolution
    return result


def visible_unknown_gain(grid, col, row, radius, free_threshold=20):
    """Unique unknown cells sampled by rays, stopped at known obstacles.

    Gain is predicted from the live map only. This approximate 360-degree
    observation model does not authorize travel into any unknown cell.
    """
    pixels = math.ceil(radius/grid.resolution)
    angles = np.arange(360)*math.tau/360
    steps = np.arange(1, pixels+1)
    cc = np.floor(col+.5+np.cos(angles[:, None])*steps).astype(np.int32)
    rr = np.floor(row+.5+np.sin(angles[:, None])*steps).astype(np.int32)
    h, w = grid.cells.shape
    inside = (cc >= 0) & (cc < w) & (rr >= 0) & (rr < h)
    values = np.full(cc.shape, -1, np.int16)
    values[inside] = grid.cells[rr[inside], cc[inside]]
    blocked = np.logical_or.accumulate(values > free_threshold, axis=1)
    unknown = (values < 0) & ~blocked
    # Padded stride keeps negative / outside-map coordinates unique.
    ids = (rr[unknown]+pixels)*(w+2*pixels)+cc[unknown]+pixels
    return len(np.unique(ids))*grid.resolution**2


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
    if not segment_is_known_free(grid, robot_xy, robot_xy, robot_radius, free_threshold):
        return Analysis([], safe, frontier_count, len(clusters), 'robot_outside_safe_space')
    _, regions = cv2.connectedComponents(safe.astype(np.uint8), connectivity=4)
    # The live pose may be safe while its containing cell centre fails the
    # conservative raster test. Connect to a nearby safe centre only after
    # validating the complete swept circle to it; never shrink the robot.
    anchor_safe = safe_cells(grid,robot_radius+grid.resolution*.1,free_threshold)
    anchors = [(rr,cc) for rr in range(max(0,row-2),min(h,row+3))
               for cc in range(max(0,col-2),min(w,col+3)) if anchor_safe[rr,cc]
               and segment_is_known_free(grid,robot_xy,grid.world(cc,rr),robot_radius,free_threshold)]
    if not anchors:
        return Analysis([], safe, frontier_count, len(clusters), 'robot_outside_safe_space')
    row, col = min(anchors,key=lambda rc: math.hypot(grid.world(rc[1],rc[0])[0]-robot_xy[0],
                                                   grid.world(rc[1],rc[0])[1]-robot_xy[1]))
    reachable = regions == regions[row, col]
    # Extra room at the observation goal prevents the final approach
    # from clipping unknown corners. Keep corridor connectivity at
    # the body radius, so this goal margin does not seal narrow doors.
    travel = travel_distances(reachable, (col, row), grid.resolution)
    goal_cells = reachable & safe_cells(grid, robot_radius+goal_clearance_margin, free_threshold)
    approach_px = math.ceil(approach_distance/grid.resolution)
    spacing_px = max(1, round(sample_spacing/grid.resolution))
    proposals = []
    excluded_count = 0
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
                excluded_count += 1
                continue
            fx, fy = grid.world(fc, fr)
            gain = visible_unknown_gain(grid, int(sc[best]), int(sr[best]),
                                        gain_radius, free_threshold)
            if gain <= 0:
                continue
            score = gain/(1.0+float(travel[sr[best], sc[best]]))
            proposals.append(Candidate(x, y, math.atan2(fy-y, fx-x), fx, fy, gain, score))
    proposals.sort(key=lambda p: (-p.score, p.x, p.y))
    selected = []
    for candidate in proposals:
        if all(math.hypot(candidate.x-p.x, candidate.y-p.y) >= .6 for p in selected):
            selected.append(candidate)
        if len(selected) >= max_candidates:
            break
    return Analysis(selected, safe, frontier_count, len(clusters),
                    'candidates' if selected else ('candidates_temporarily_blacklisted'
                    if excluded_count else 'no_reachable_frontiers'), grid.world(col,row))


def path_is_safe(grid, safe, points, robot_radius=.47, free_threshold=20):
    """Validate continuous swept circles, including sub-cell pose offsets."""
    if not points or safe.shape != grid.cells.shape:
        return False
    return all(segment_is_known_free(grid,a,b,robot_radius,free_threshold)
               for a,b in zip(points,points[1:] or points))


def remaining_path(points, pose):
    """Project onto a polyline and omit the section already traversed."""
    if len(points) < 2:return [pose]+points
    best = None
    for i,(a,b) in enumerate(zip(points,points[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
        t = max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/length)) if length else 0.
        q=(a[0]+t*dx,a[1]+t*dy)
        distance=(pose[0]-q[0])**2+(pose[1]-q[1])**2
        if best is None or distance < best[0]:best=(distance,i,q)
    return [pose,best[2]]+points[best[1]+1:]
