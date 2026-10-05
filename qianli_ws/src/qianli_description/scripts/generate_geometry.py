#!/usr/bin/env python3
"""Generate a 28-triangle convex plate STL and Nav2 footprint, offline.

The numeric properties in urdf/common.xacro are the only dimension source.
Only the Python standard library is needed. --check never writes files.
"""

import argparse
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


def dimensions(package):
    root = ET.parse(package / 'urdf/common.xacro').getroot()
    return {p.attrib['name']: float(p.attrib['value'])
            for p in root.findall('{http://www.ros.org/wiki/xacro}property')
            if not p.attrib['value'].startswith('${')}


def vertices(p):
    size, cut = p['base_size'], p['base_cut']
    if not (0 < cut < size / 2 and p['base_thickness'] > 0):
        raise ValueError('Require 0 < base_cut < base_size/2 and positive thickness')
    if not math.isclose(size - 2 * cut, p['base_long_edge'], abs_tol=1e-12):
        raise ValueError('base_long_edge must equal base_size - 2*base_cut')
    a, b = size / 2, size / 2 - cut
    return [(a, b), (b, a), (-b, a), (-a, b),
            (-a, -b), (-b, -a), (b, -a), (a, -b)]


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def mesh_bytes(p):
    xy = vertices(p)
    low = [(x, y, -p['base_thickness']/2) for x, y in xy]
    high = [(x, y, p['base_thickness']/2) for x, y in xy]
    triangles = []
    for i in range(1, 7):
        triangles.extend([(high[0], high[i], high[i+1]),
                          (low[0], low[i+1], low[i])])
    for i in range(8):
        j = (i+1) % 8
        triangles.extend([(low[i], low[j], high[j]), (low[i], high[j], high[i])])
    data = bytearray(b'QianLi Base Geometry v0.1; metres; generated from common.xacro'.ljust(80, b'\0'))
    data.extend(struct.pack('<I', len(triangles)))
    for a, b, c in triangles:
        n = cross(tuple(y-x for x, y in zip(a, b)), tuple(y-x for x, y in zip(a, c)))
        length = math.sqrt(sum(x*x for x in n))
        data.extend(struct.pack('<12fH', *(x/length for x in n), *a, *b, *c, 0))
    return bytes(data)


def footprint_bytes(p):
    polygon = json.dumps([[round(x, 12), round(y, 12)] for x, y in vertices(p)])
    text = ('# Generated from urdf/common.xacro; regenerate with scripts/generate_geometry.py.\n'
            '# Plate outline requested for Geometry v0.1; wheel protrusion is not included.\n'
            '# Nav2 uses a string parameter for a polygon, not a nested ROS parameter array.\n')
    for name in ('local_costmap', 'global_costmap'):
        text += (f'{name}:\n  {name}:\n    ros__parameters:\n'
                 f'      robot_base_frame: base_footprint\n      footprint: "{polygon}"\n')
    return text.encode('utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    p = dimensions(package)
    assets = {package / 'meshes/base/qianli_base.stl': mesh_bytes(p),
              package / 'config/nav2_footprint.yaml': footprint_bytes(p)}
    for path, data in assets.items():
        if args.check:
            if not path.exists() or path.read_bytes() != data:
                raise SystemExit(f'Stale/missing {path}. Run python3 scripts/generate_geometry.py')
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        print(f'{"Verified" if args.check else "Generated"}: {path.relative_to(package)}')


if __name__ == '__main__':
    main()
