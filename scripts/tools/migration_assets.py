#!/usr/bin/env python3
"""Export and verify robot assets without starting ROS or opening hardware.

Run export on the old machine, transfer the bundle, then verify/restore on the
new machine. Restoration rejects unsafe paths and changed existing files.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
import subprocess
import tempfile
import zipfile

from project_paths import calibration_path, project_root

CALIB_FILES = (
    'camera_intrinsics.yaml', 'extrinsic.txt', 'grasp_offset.txt', 'tcp_calib.txt',
    'camera_ref.npz', 'board_homography.json', 'homography.npy', 'board_bg.png',
    'board_bg_color.png', 'extrinsic_marks.json', 'extrinsic_marks_merged.json',
    'joint_ranges.json', 'joint_ranges_merged.json', 'homing_backup.json',
    'table_limit.txt', 'table_marks.json', 'tcp_marks.json', 'pinch_offset.npy',
)
ALLOWED_SUFFIXES = {'.py', '.xml', '.cfg', '.yaml', '.rviz', '.urdf', '.stl', '.json', '.npz', '.npy', '.png', '.txt'}


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def safe_name(name):
    path = PurePosixPath(name)
    if not name or path.is_absolute() or '..' in path.parts or ':' in name or '\\' in name:
        raise ValueError(f'Unsafe bundle path: {name}')
    return path


def export_assets(output, legacy_root, calib_dir):
    root = project_root()
    live_source = legacy_root / 'src/so101_bringup'
    mirror = root / 'qianli_ws/log/vm_mirror/so101_bringup'
    canonical = root/'qianli_ws/src/qianli_arm'
    if live_source.is_dir():
        source=live_source
    elif (canonical/'package.xml').is_file():
        source=canonical
    elif mirror.is_dir():
        source=mirror
    else:
        raise ValueError('Complete SO-101 driver source not found')
    items = {}

    def add(name, path):
        if path.is_file():
            safe_name(name)
            items[name] = path

    for base, label in ((source, 'driver-source'), (root/'config', 'project-config'),
                        (root/'qianli_ws/config', 'legacy-project-config'),
                        (root/'calib', 'calibration-candidates/project')):
        if base.is_dir():
            for path in sorted(base.rglob('*')):
                if '__pycache__' in path.parts or any(x in path.parts for x in ('build', 'install', '.git')):
                    continue
                if path.suffix in ALLOWED_SUFFIXES or path.parent.name == 'resource':
                    add(label+'/'+path.relative_to(base).as_posix(), path)
    installed_config = legacy_root / 'install/so101_bringup/share/so101_bringup/config/driver_params.yaml'
    add('runtime-config/driver_params.yaml', installed_config)
    # Preserve the complete persistent calibration directory and servo rollback
    # records, including new filenames unknown to this migration script.
    if calib_dir.is_dir():
        for path in sorted(calib_dir.rglob('*')):
            if path.suffix in ALLOWED_SUFFIXES:
                add('calibration/'+path.relative_to(calib_dir).as_posix(), path)
    for label in ('limit_fix_backup', 'homing_backup', 'grasp_audit'):
        base = root/'qianli_ws/log'/label
        if base.is_dir():
            for path in sorted(base.rglob('*')):
                if path.is_file():
                    add('safety-records/'+label+'/'+path.relative_to(base).as_posix(), path)
    missing = []
    for name in CALIB_FILES:
        candidates = [calib_dir/name, Path(tempfile.gettempdir())/name]
        available = [p for p in candidates if p.is_file()]
        if available:
            add('calibration/'+name, available[0])
            for i, candidate in enumerate(available[1:]):
                add(f'calibration-candidates/tmp-{i}/{name}', candidate)
        else:
            missing.append(name)
    for path in sorted((root/'dual_twin').rglob('*')):
        if 'scripts' not in path.parts and path.suffix in {'.urdf', '.stl', '.txt', '.zip'}:
            add('simulation/'+path.relative_to(root/'dual_twin').as_posix(), path)
    model = root/'qianli_ws/src/qianli_description'
    if model.is_dir():
        for path in sorted(model.rglob('*')):
            if path.suffix in ALLOWED_SUFFIXES:
                add('ros-model/'+path.relative_to(model).as_posix(), path)
    parts = Path.home()/'mj_parts'
    if parts.is_dir():
        for path in sorted(parts.rglob('*')):
            if path.suffix in {'.stl', '.txt', '.json'}:
                add('legacy-convex-parts/'+path.relative_to(parts).as_posix(), path)
    try:
        revision = subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip()
    except (OSError, subprocess.SubprocessError):
        revision = None
    manifest = {'format': 1, 'hostname': platform.node(), 'revision': revision,
                'created_utc': datetime.now(timezone.utc).isoformat(),
                'driver_source': str(source), 'live_legacy_source': source == live_source,
                'missing_calibration': missing, 'files': {}}
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
        for name, path in items.items():
            data = path.read_bytes()
            archive.writestr(name, data)
            manifest['files'][name] = {'sha256': sha256(data), 'size': len(data), 'source': str(path)}
        archive.writestr('manifest.json', json.dumps(manifest,ensure_ascii=False,indent=2))
    print(json.dumps({'bundle': str(output), 'files': len(items),
                      'live_legacy_source': manifest['live_legacy_source'],
                      'missing_calibration': missing},ensure_ascii=False,indent=2))


def verify_assets(bundle, require_calibration=False):
    with zipfile.ZipFile(bundle) as archive:
        names = archive.namelist()
        if len(names) != len(set(names)):
            raise ValueError('Duplicate archive members')
        for name in names:
            safe_name(name)
        manifest = json.loads(archive.read('manifest.json'))
        if manifest.get('format') != 1:
            raise ValueError('Unsupported manifest format')
        if set(names) != set(manifest['files']) | {'manifest.json'}:
            raise ValueError('Archive members differ from manifest')
        for name, entry in manifest['files'].items():
            data = archive.read(name)
            if len(data) != entry['size'] or sha256(data) != entry['sha256']:
                raise ValueError(f'Checksum mismatch: {name}')
        if require_calibration:
            required = {'calibration/camera_intrinsics.yaml','calibration/extrinsic.txt'}
            if not required <= set(names):
                raise ValueError('Required camera intrinsics/extrinsics are missing')
    return manifest


def restore_assets(bundle, destination, calib_dir):
    manifest = verify_assets(bundle)
    destinations = {}
    with zipfile.ZipFile(bundle) as archive:
        manifest_path = destination/'manifest.json'
        manifest_bytes = (json.dumps(manifest,ensure_ascii=False,indent=2)+'\n').encode('utf-8')
        if not manifest_path.resolve().is_relative_to(destination.resolve()):
            raise ValueError('Manifest path escapes destination')
        if manifest_path.exists() and manifest_path.read_bytes().rstrip() != manifest_bytes.rstrip():
            raise ValueError(f'Existing manifest differs: {manifest_path}')
        destinations[manifest_path] = manifest_bytes
        for name in manifest['files']:
            relative = safe_name(name)
            targets = [destination.joinpath(*relative.parts)]
            if relative.parts[0] == 'calibration':
                targets.append(calib_dir.joinpath(*relative.parts[1:]))
            data = archive.read(name)
            for target in targets:
                boundary = calib_dir.resolve() if target in targets[1:] else destination.resolve()
                resolved = target.resolve()
                if not resolved.is_relative_to(boundary):
                    raise ValueError(f'Path escapes destination: {target}')
                if target.exists() and (not target.is_file() or target.read_bytes() != data):
                    raise ValueError(f'Existing file differs; preserve it and choose another directory: {target}')
                destinations[target] = data
        # Preflight every destination before writing any file.
        for target, data in destinations.items():
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists():
                with target.open('xb') as stream:
                    stream.write(data)
    print(f'Restored {len(destinations)} files; validate calibration before real use')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    export = sub.add_parser('export')
    export.add_argument('--output', type=Path, required=True)
    export.add_argument('--legacy-root', type=Path, default=Path.home()/'legacy/arm/arm-final/ros2_ws')
    export.add_argument('--calib-dir', type=Path, default=Path(calibration_path()))
    verify = sub.add_parser('verify')
    verify.add_argument('bundle', type=Path)
    verify.add_argument('--require-calibration', action='store_true')
    restore = sub.add_parser('restore')
    restore.add_argument('bundle', type=Path)
    restore.add_argument('--destination', type=Path, required=True)
    restore.add_argument('--calib-dir', type=Path, default=Path(calibration_path()))
    args = parser.parse_args()
    try:
        if args.command == 'export':
            export_assets(args.output, args.legacy_root.expanduser(), args.calib_dir.expanduser())
        elif args.command == 'verify':
            manifest = verify_assets(args.bundle, args.require_calibration)
            print(json.dumps({'verified_files':len(manifest['files']),
                              'missing_calibration':manifest['missing_calibration']},ensure_ascii=False,indent=2))
        else:
            restore_assets(args.bundle, args.destination.expanduser(), args.calib_dir.expanduser())
    except (OSError, ValueError, KeyError, zipfile.BadZipFile) as exc:
        parser.exit(1, f'[ERROR] {exc}\n')


if __name__ == '__main__':
    main()
