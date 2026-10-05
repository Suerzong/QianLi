#!/usr/bin/env python3
"""Launch only this teaching-scene session; do not alter other ROS sessions."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import signal
import time

parser = argparse.ArgumentParser()
parser.add_argument('mode', choices=['start', 'benchmark', 'status', 'stop'])
args = parser.parse_args()
ws = Path('/home/ros/QianLi/qianli_ws')
logs = ws / 'log/teaching_training_v1'
logs.mkdir(parents=True, exist_ok=True)
env = os.environ.copy()
env.update(ROS_DOMAIN_ID='78', GZ_PARTITION='qianli_teaching_training_v1',
           DISPLAY=':98', LIBGL_ALWAYS_SOFTWARE='1', QT_QPA_PLATFORM='xcb',
           XDG_RUNTIME_DIR='/run/user/1000')
env.pop('XAUTHORITY', None)
setup = 'source /opt/ros/jazzy/setup.bash\nsource /home/ros/QianLi/qianli_ws/install/setup.bash\n'

def group_members(pgid):
    members = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if os.getpgid(int(proc.name)) == pgid and (proc/'stat').read_text().split()[2] != 'Z':
                members.append(proc)
        except (ProcessLookupError, FileNotFoundError, PermissionError):
            pass
    return members

def check_identity(members):
    for proc in members:
        variables = (proc/'environ').read_bytes().split(b'\0')
        if b'ROS_DOMAIN_ID=78' not in variables or b'GZ_PARTITION=qianli_teaching_training_v1' not in variables:
            raise RuntimeError('Session identity mismatch: ' + proc.name)

def start(name, command):
    pidfile = logs / (name + '.pid')
    if pidfile.exists() and group_members(int(pidfile.read_text())):
        raise RuntimeError(name + ' is already running')
    with (logs / (name + '.log')).open('w') as stream:
        process = subprocess.Popen(['bash', '-lc', setup + 'exec ' + command],
                                   cwd=ws, env=env, start_new_session=True,
                                   stdin=subprocess.DEVNULL, stdout=stream,
                                   stderr=subprocess.STDOUT)
    pidfile.write_text(str(process.pid))
    print(name, process.pid, flush=True)

if args.mode == 'start':
    display_check = subprocess.run(['xdpyinfo', '-display', env['DISPLAY']],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if display_check.returncode != 0:
        with (logs / 'xvfb.log').open('w') as stream:
            display = subprocess.Popen(['Xvfb', ':98', '-screen', '0', '1600x1000x24', '-ac'],
                                       stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
        (logs / 'xvfb.pid').write_text(str(display.pid))
        for _ in range(50):
            if subprocess.run(['xdpyinfo', '-display', ':98'], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL).returncode == 0:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Xvfb did not become ready; refusing broken GPU lidar startup')
    start('bringup', 'ros2 launch qianli_bringup training.launch.py rviz:=true')
elif args.mode == 'benchmark':
    start('benchmark', 'ros2 run qianli_training_scenarios benchmark.py '
          '--manifest /home/ros/QianLi/qianli_ws/src/qianli_training_scenarios/generated/baseline/manifest.json '
          '--output /home/ros/QianLi/qianli_ws/log/teaching_training_v1/smoke.json '
          '--suite smoke --timeout 360 --telemetry-timeout 8')
elif args.mode == 'status':
    for name in ['bringup', 'benchmark', 'gui']:
        path = logs / (name + '.pid')
        if path.exists():
            pid = path.read_text().strip()
            print(name, pid, 'group_members', len(group_members(int(pid))))
    path = logs / 'smoke.json'
    if path.exists():
        report = json.loads(path.read_text())
        print(json.dumps({'status': report['status'], 'passed_tasks': report.get('passed_tasks'),
                          'tasks': [{k: task.get(k) for k in ['id','status','terminal_reason',
                                     'final_position_error_m','action_status_name']}
                                    for task in report['tasks']]}, indent=2))
elif args.mode == 'stop':
    for name in ['benchmark', 'gui', 'bringup']:
        pidfile = logs / (name + '.pid')
        if not pidfile.exists():
            continue
        pid = int(pidfile.read_text())
        members = group_members(pid)
        if not members:
            continue
        check_identity(members)
        os.killpg(pid, signal.SIGINT if name == 'bringup' else signal.SIGTERM)
        deadline = time.monotonic() + 10.
        while group_members(pid) and time.monotonic() < deadline:
            time.sleep(.1)
        remaining = group_members(pid)
        if remaining:
            check_identity(remaining)
            os.killpg(pid, signal.SIGTERM)
        print('stopped', name, pid)
