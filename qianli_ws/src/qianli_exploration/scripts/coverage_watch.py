#!/usr/bin/env python3
"""Read-only simulation coverage sidecar; truth never enters exploration."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import yaml
from frontier_core import Grid
from coverage_core import reachable_domain, map_coverage


def yaw(q):
    return np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class CoverageWatch(Node):
    def __init__(self, manifest, output):
        super().__init__('qianli_coverage_watch')
        self.output = output
        data = json.loads(manifest.read_text())
        info = yaml.safe_load((manifest.parent/'teaching.yaml').read_text())
        image = cv2.imread(str(manifest.parent/info['image']), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError('Reference map is missing')
        # PGM row zero is the highest y; OccupancyGrid row zero is the lowest.
        self.reference = Grid(np.where(np.flipud(image) > 250, 0, 100).astype(np.int16),
                              info['resolution'], *info['origin'])
        self.spawn = data['spawn'][:3]
        # Generated manifest spawn is [x, y, yaw], not [x, y, z].
        self.domain = reachable_domain(self.reference, self.spawn, bounds=data['bounds'])
        self.current = self.status = None
        self.history = []
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latch)
        self.create_subscription(String, '/exploration/status', self.on_status, latch)
        self.create_timer(5., self.record, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_map(self, msg):
        if msg.header.frame_id == 'map' and msg.data:
            grid = Grid(np.asarray(msg.data).reshape(msg.info.height, msg.info.width),
                        msg.info.resolution, msg.info.origin.position.x,
                        msg.info.origin.position.y, float(yaw(msg.info.origin.orientation)))
            self.current = map_coverage(self.reference, self.domain, grid, self.spawn)

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def record(self):
        if self.current is None or self.status is None:
            return
        self.history.append({'elapsed_s': self.status['wall_elapsed_s'],
                             'free_coverage_pct': self.current['free_coverage_pct'],
                             'goals': self.status['successful_goals'],
                             'phase': self.status['phase'], 'reason': self.status['reason']})
        result = {**self.current, 'status': {k:self.status[k] for k in
                  ('phase','reason','terminal','successful_goals','failed_candidates','known_cells')},
                  'history': self.history}
        self.output.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.output.with_suffix('.tmp')
        tmp.write_text(json.dumps(result, indent=2)+'\n')
        tmp.replace(self.output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = CoverageWatch(args.manifest, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.record()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
