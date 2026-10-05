#!/usr/bin/env python3
"""Validate expanded URDF transforms, primitive sizes, mesh and footprint."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET

from generate_geometry import cross, dimensions, vertices


def close(actual, expected, tol=1e-7):
    if isinstance(expected, (list, tuple)):
        assert len(actual) == len(expected), (actual, expected)
        for a, e in zip(actual, expected):
            close(a, e, tol)
    else:
        assert math.isclose(actual, expected, abs_tol=tol, rel_tol=tol), (actual, expected)


def vector(element, attribute, default='0 0 0'):
    return tuple(map(float, element.get(attribute, default).split()))


def rotate(rpy, v):
    r, p, y = rpy
    x1, y1, z1 = v[0], math.cos(r)*v[1]-math.sin(r)*v[2], math.sin(r)*v[1]+math.cos(r)*v[2]
    x2, y2, z2 = math.cos(p)*x1+math.sin(p)*z1, y1, -math.sin(p)*x1+math.cos(p)*z1
    return (math.cos(y)*x2-math.sin(y)*y2, math.sin(y)*x2+math.cos(y)*y2, z2)


def validate_urdf(root, p):
    signs = {'front_left': (1, 1), 'front_right': (1, -1),
             'rear_left': (-1, 1), 'rear_right': (-1, -1)}
    links = {e.get('name'): e for e in root.findall('link')}
    joints = {e.get('name'): e for e in root.findall('joint')}
    assert len(root.findall('link')) == len(links) == 6
    assert len(root.findall('joint')) == len(joints) == 5
    assert set(links) == {'base_footprint', 'base_link'} | {n+'_wheel_link' for n in signs}
    children = Counter(j.find('child').get('link') for j in joints.values())
    assert set(links) - set(children) == {'base_footprint'}
    assert all(c == 1 for c in children.values())
    assert not root.findall('.//inertial')
    assert not root.findall('.//transmission') and not root.findall('.//ros2_control')
    assert not root.findall('.//gazebo')
    base = joints['base_footprint_joint']
    assert base.get('type') == 'fixed'
    assert base.find('parent').get('link') == 'base_footprint'
    assert base.find('child').get('link') == 'base_link'
    close(vector(base.find('origin'), 'xyz'), (0, 0, p['base_height']))
    close(vector(base.find('origin'), 'rpy'), (0, 0, 0))
    for kind in ('visual', 'collision'):
        element = links['base_link'].find(kind)
        mesh = element.find('geometry/mesh')
        assert mesh.get('filename') == 'package://qianli_description/meshes/base/qianli_base.stl'
        close(tuple(map(float, mesh.get('scale', '1 1 1').split())), (1, 1, 1))
        if element.find('origin') is not None:
            close(vector(element.find('origin'), 'xyz'), (0, 0, 0))
            close(vector(element.find('origin'), 'rpy'), (0, 0, 0))
    for name, (sx, sy) in signs.items():
        joint = joints[name+'_wheel_joint']
        assert joint.get('type') == 'continuous'
        assert joint.find('parent').get('link') == 'base_link'
        assert joint.find('child').get('link') == name+'_wheel_link'
        xyz = vector(joint.find('origin'), 'xyz')
        rpy = vector(joint.find('origin'), 'rpy')
        close(xyz, (sx*p['wheel_x'], sy*p['wheel_y'], p['wheel_z']-p['base_height']))
        axis = vector(joint.find('axis'), 'xyz')
        close(axis, (0, 0, 1))
        world_axis = rotate(rpy, axis)
        close(world_axis, (sx/math.sqrt(2), sy/math.sqrt(2), 0))
        for kind in ('visual', 'collision'):
            element = links[name+'_wheel_link'].find(kind)
            cylinder = element.find('geometry/cylinder')
            close(float(cylinder.get('radius')), p['wheel_radius'])
            close(float(cylinder.get('length')), p['wheel_width'])
            local_rpy = (0, 0, 0)
            if element.find('origin') is not None:
                close(vector(element.find('origin'), 'xyz'), (0, 0, 0))
                local_rpy = vector(element.find('origin'), 'rpy')
            close(rotate(rpy, rotate(local_rpy, (0, 0, 1))), world_axis)
        close(xyz[2]+p['base_height']-p['wheel_radius'], 0)
        print(f'{name}: ground xyz=({xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]+p["base_height"]:.6f}), axis={world_axis}')
    print('PASS: one connected TF tree; 6 links, 5 joints; wheel positions/axes/ground contact')


def validate_mesh(package, p):
    data = (package / 'meshes/base/qianli_base.stl').read_bytes()
    count, = struct.unpack_from('<I', data, 80)
    assert count == 28 and len(data) == 84+50*count
    points, edges, volume = set(), Counter(), 0.0
    for i in range(count):
        values = struct.unpack_from('<12fH', data, 84+50*i)
        n, a, b, c = [tuple(values[j:j+3]) for j in (0, 3, 6, 9)]
        points.update((a, b, c))
        for v, w in ((a, b), (b, c), (c, a)):
            edges[(v, w)] += 1
        normal = cross(tuple(v-u for u, v in zip(a, b)), tuple(v-u for u, v in zip(a, c)))
        length = math.sqrt(sum(v*v for v in normal))
        assert length > 0
        close(n, tuple(v/length for v in normal))
        center = tuple((u+v+w)/3 for u, v, w in zip(a, b, c))
        assert sum(u*v for u, v in zip(n, center)) > 0, 'inward normal'
        volume += sum(u*v for u, v in zip(a, cross(b, c)))/6
    assert len(points) == 16
    assert all(c == 1 and edges[(b, a)] == 1 for (a, b), c in edges.items()), 'not watertight'
    actual = sorted(points)
    expected = sorted((x, y, z) for x, y in vertices(p)
                      for z in (-p['base_thickness']/2, p['base_thickness']/2))
    for a, b in zip(actual, expected):
        close(a, b)
    close(volume, (p['base_size']**2-2*p['base_cut']**2)*p['base_thickness'])
    xy = vertices(p)
    lengths = [math.dist(xy[i], xy[(i+1) % 8]) for i in range(8)]
    close(lengths, [math.sqrt(2)*p['base_cut'], p['base_long_edge']]*4)
    print(f'PASS: closed outward-facing convex STL, {count} triangles, volume={volume:.9f} m^3; edges={lengths}')


def validate_footprint(package, p):
    import yaml
    config = yaml.safe_load((package / 'config/nav2_footprint.yaml').read_text())
    for name in ('local_costmap', 'global_costmap'):
        params = config[name][name]['ros__parameters']
        assert params['robot_base_frame'] == 'base_footprint'
        assert isinstance(params['footprint'], str)
        polygon = json.loads(params['footprint'])
        for actual, expected in zip(polygon, vertices(p)):
            close(actual, expected)
        assert len(polygon) == 8
    print('PASS: local/global Nav2 polygon strings match plate geometry')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, help='Validate an already expanded URDF')
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    p = dimensions(package)
    xml = (args.urdf.read_text() if args.urdf else subprocess.check_output(
        ['xacro', str(package / 'urdf/qianli.urdf.xacro')], text=True))
    validate_urdf(ET.fromstring(xml), p)
    validate_mesh(package, p)
    validate_footprint(package, p)
    subprocess.run(['python3', str(package / 'scripts/generate_geometry.py'), '--check'], check=True)
    print('PASS: QianLi Base Geometry v0.1 validation')


if __name__ == '__main__':
    main()
