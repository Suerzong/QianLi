#!/usr/bin/env python3
"""Independent, bounded exploration smoke check; truth never reaches the policy.

Verifies real map growth, completed autonomous goals, physical travel and sampled
outline/padded geometry. A partial smoke pass does not mean whole-floor coverage.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool

sys.path.insert(0, str(Path(get_package_share_directory('qianli_training_scenarios')) / 'scripts'))
from benchmark import (RAW_FOOTPRINT, PADDED_FOOTPRINT, transform_polygon,
                       prepare_obstacles, footprint_clearance)


def obstacle_manifest(world):
    boxes = []
    for model in ET.parse(world).findall('world/model'):
        if model.get('name') in ('ground', 'floor'):
            continue
        mp = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
        for link in model.findall('link'):
            lp = list(map(float, link.findtext('pose', '0 0 0 0 0 0').split()))
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if size is None:
                    continue
                cp = list(map(float, collision.findtext('pose', '0 0 0 0 0 0').split()))
                sx, sy, sz = map(float, size.split())
                mx, my = mp[:2]
                lx, ly = transform_point(lp[0], lp[1], mx, my, mp[5])
                x, y = transform_point(cp[0], cp[1], lx, ly, mp[5]+lp[5])
                if mp[2]+lp[2]+cp[2]+sz/2 <= .05:
                    continue
                if any(abs(p[i]) > 1e-6 for p in (mp, lp, cp) for i in (3, 4)):
                    raise ValueError('Smoke scorer requires upright box obstacles')
                boxes.append({'id': model.get('name')+'/'+link.get('name')+'/'+collision.get('name'),
                              'x': x, 'y': y, 'sx': sx, 'sy': sy, 'yaw': mp[5]+lp[5]+cp[5]})
    return {'obstacles': boxes}


def transform_point(x, y, ox, oy, angle):
    c, s = math.cos(angle), math.sin(angle)
    return ox+c*x-s*y, oy+s*x+c*y


class Observer(Node):
    def __init__(self, obstacles):
        super().__init__('qianli_exploration_smoke')
        self.obstacles = obstacles
        self.status = None
        self.known_start = self.free_start = None
        self.known = self.free = 0
        self.path_length = 0.
        self.last_pose = None
        self.truth_samples = self.geometry_samples = 0
        self.raw_collisions = self.padded_collisions = 0
        self.min_padded_clearance = None
        self.last_geometry_wall = 0.
        self.last_status_wall = self.last_map_wall = self.last_truth_wall = 0.
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/exploration/status', self.on_status, latch)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latch)
        self.create_subscription(Odometry, '/simulation/ground_truth', self.on_truth, qos_profile_sensor_data)
        self.stop_client = self.create_client(SetBool, '/exploration/enable')

    def on_status(self, message):
        self.status = json.loads(message.data)
        self.last_status_wall = time.monotonic()

    def on_map(self, message):
        self.known = sum(v >= 0 for v in message.data)
        self.free = sum(0 <= v <= 20 for v in message.data)
        if self.known_start is None:
            self.known_start, self.free_start = self.known, self.free
        self.last_map_wall = time.monotonic()

    def on_truth(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = (p.x, p.y, heading)
        if self.last_pose:
            self.path_length += math.hypot(p.x-self.last_pose[0], p.y-self.last_pose[1])
        self.last_pose = pose
        self.last_truth_wall = time.monotonic()
        self.truth_samples += 1
        if self.last_truth_wall-self.last_geometry_wall < .15:
            return
        self.last_geometry_wall = self.last_truth_wall
        raw, _, hits = footprint_clearance(transform_polygon(RAW_FOOTPRINT, pose), self.obstacles)
        padded, _, padded_hits = footprint_clearance(transform_polygon(PADDED_FOOTPRINT, pose), self.obstacles)
        self.raw_collisions += bool(hits)
        self.padded_collisions += bool(padded_hits)
        self.geometry_samples += 1
        self.min_padded_clearance = padded if self.min_padded_clearance is None else min(self.min_padded_clearance, padded)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument('--world', type=Path)
    source.add_argument('--manifest', type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--duration', type=float, default=240.)
    ap.add_argument('--min-gain-cells', type=int, default=1000)
    ap.add_argument('--min-goals', type=int, default=2)
    ap.add_argument('--min-path-m', type=float, default=1.)
    ap.add_argument('--require-exhausted', action='store_true')
    args = ap.parse_args()
    manifest = json.loads(args.manifest.read_text()) if args.manifest else obstacle_manifest(args.world)
    obstacles = prepare_obstacles(manifest)
    rclpy.init()
    node = Observer(obstacles)
    started = time.monotonic()
    before_stop = None
    reason = 'timeout'
    try:
        while rclpy.ok() and time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
            if node.raw_collisions or node.padded_collisions:
                reason = 'geometry_intersection'
                break
            status = node.status
            if status is None or node.known_start is None:
                continue
            growth = node.known-node.known_start
            passed = (status['successful_goals'] >= args.min_goals and growth >= args.min_gain_cells
                      and node.path_length >= args.min_path_m and node.truth_samples >= 20)
            if passed and (not args.require_exhausted or status['terminal'] == 'exhausted'):
                reason = 'exhausted_verified' if status['terminal'] == 'exhausted' else 'growth_verified'
                break
            if status['terminal']:
                reason = 'premature_'+status['terminal']
                break
            if time.monotonic()-started > 30 and any(time.monotonic()-stamp > 12 for stamp in
                                                     (node.last_status_wall, node.last_map_wall, node.last_truth_wall)):
                reason = 'telemetry_stale'
                break
    finally:
        before_stop = node.status
        confirmation = False
        if node.stop_client.wait_for_service(timeout_sec=3):
            future = node.stop_client.call_async(SetBool.Request(data=False))
            deadline = time.monotonic()+8
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if future.done() and node.status and node.status['phase'] == 'stopped':
                    confirmation = True
                    break
        if node.raw_collisions or node.padded_collisions:
            reason = 'geometry_intersection'
        report = {
            'result': 'PASS' if reason in ('growth_verified', 'exhausted_verified') and confirmation else 'FAIL',
            'reason': reason, 'stop_confirmed': confirmation,
            'wall_elapsed_s': round(time.monotonic()-started, 3),
            'initial_known_cells': node.known_start, 'final_known_cells': node.known,
            'known_gain_cells': node.known-(node.known_start or 0),
            'free_gain_cells': node.free-(node.free_start or 0),
            'path_length_m': node.path_length, 'truth_samples': node.truth_samples,
            'geometry_samples': node.geometry_samples, 'raw_collision_samples': node.raw_collisions,
            'padded_collision_samples': node.padded_collisions, 'min_padded_clearance_m': node.min_padded_clearance,
            'exploration_before_stop': before_stop, 'exploration_after_stop': node.status,
            'scope': 'Partial exploration smoke check unless exhausted_verified; sampled ideal geometry, no wheel dynamics',
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        print(json.dumps({k: v for k, v in report.items() if not k.startswith('exploration_')}, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
