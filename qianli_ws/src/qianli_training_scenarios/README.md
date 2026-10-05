# QianLi Teaching Training v1

Metric synthetic single-floor teaching building for ROS 2 Jazzy / Gazebo
Harmonic navigation evaluation. The reference photograph informs functional
layout; ALL building dimensions are inferred design choices. This supersedes
the photo-threshold maps for training. No original image pixels are used.

## Geometry and data

- Building: 48 x 40 m. Origin at building centre, +X east/right, +Y north/up,
  +Z up. Map includes 0.5 m exterior border.
- Wall thickness 0.16 m, height 2.6 m; ordinary doors net width 1.20 m,
  lecture entrances 1.80 m, hall connections 2.40 m.
- West corridor width 3.0 m; auditorium east/north/south circulation about
  4.0 m. Room doors and seating aisles are intentional navigation challenges.
- Eight repeated classrooms, six office/lab/seminar rooms, two lecture halls,
  desk/seat blocks, pillars, reception and benches. Closed stairs and lift
  are intentionally inaccessible on this floor.
- Map resolution 0.05 m/px, origin [-24.5,-20.5,0], 980 x 820 cells.
- QianLi plate footprint is the existing 0.70 m octagon with 0.06 m padding.
  The footprint follows Geometry v0.1 and excludes wheel protrusion.
- Spawn [13.5,-16.0,0] in main lobby. AMCL initial pose equals spawn;
  controller odom starts at its local zero and AMCL supplies map to odom.

`scripts/generate_scene.py` defines every wall opening and obstacle in metres.
It writes `manifest.json`, `teaching.pgm/.yaml`, `teaching.sdf`, `layout.png`,
and `validation.json` into `generated/<variant>/` from the SAME object list.
Each collision box has the exact dimensions and pose of its map polygon.
The occupancy raster conservatively rounds box outlines within one 0.05 m cell.
One static Gazebo model/link contains all building objects (no duplicated
Hough wall fragments). Floor is excluded from the 2D obstacle layer.

Variants: baseline, train_000, train_001, test_101. Changes use fixed random
seeds; obstacle carts are included in each variant's map. The held-out test
layout includes an additional partial corridor restriction. These provide
repeatable evaluation datasets, not a learned robot policy.

Regenerate with Python3 + NumPy + Pillow + OpenCV:

```bash
python3 scripts/generate_scene.py
```

Validation compares SDF box dimensions and poses against the manifest and
checks start/goal connectivity with a conservative rotating padded-body
clearance. Only T10 intentionally has an unreachable destination.

## Run in the existing QianLi workspace

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ros/QianLi/qianli_ws
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=78
export GZ_PARTITION=qianli_teaching_training_v1
# This VM uses software OpenGL on the existing Xvfb display.
export DISPLAY=:98 LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb
if ! xdpyinfo -display :98 >/dev/null 2>&1; then
  Xvfb :98 -screen 0 1600x1000x24 -ac >/tmp/qianli_xvfb.log 2>&1 &
  sleep 1
fi
ros2 launch qianli_bringup training.launch.py rviz:=true
```

Choose `variant:=test_101` or `slam:=true` for the alternate geometry or
online SLAM. SLAM disables saved-map AMCL so map->odom has one owner.
Map/world overrides are optional; keep them paired.

Run the continuous three-goal smoke route in a second terminal on Domain78:

```bash
SHARE=$(ros2 pkg prefix qianli_training_scenarios)/share/qianli_training_scenarios
ros2 run qianli_training_scenarios benchmark.py \
  --manifest "$SHARE/generated/baseline/manifest.json" \
  --output /tmp/qianli_training_smoke.json --suite smoke --timeout 360
```

Smoke route enters the smaller lecture hall, crosses its 1.20 m side door,
then enters the project room through another 1.20 m doorway. The full suite
adds long/repeated corridors, turns, pillar avoidance, auditorium seating,
loop routes and a closed-stairs goal. Full-suite repositioning is performed
with navigation and recorded separately; no teleporting is used.
For a repeatable smoke trial, stop/restart the scene to restore its initial
pose before rerunning benchmark. A mismatched smoke start is reported as a
failure; it is never silently counted as the declared route.

Reports include action status, physical Gazebo endpoint error, actual path
length, outline/padded collisions, clearance and AMCL error when timestamped
data is available. Benchmark always writes a report, cancels outstanding
goals and sends zero velocity at termination.

Run all ten planning cases (without moving the robot):

```bash
ros2 run qianli_training_scenarios plan_suite.py \
  --manifest "$SHARE/generated/baseline/manifest.json" \
  --output /tmp/qianli_training_plans.json
```

T10 is accepted as unreachable only when ComputePathToPose explicitly returns
ABORTED with NO_VALID_PATH. Other failures are recorded as failures.

The VM's :98 display is ephemeral after reboot. GPU lidar needs a working X
server before Gazebo starts. A missing display can produce all-infinity scans
and invalid localization even though navigation actions report success.
Check sustained finite scans before evaluating results.

For this VM an optional session wrapper starts/checks :98 automatically and
isolates Domain78. It writes PIDs and reports in `qianli_ws/log/teaching_training_v1`:

```bash
ros2 run qianli_training_scenarios run_vm_session.py start
ros2 run qianli_training_scenarios run_vm_session.py benchmark
ros2 run qianli_training_scenarios run_vm_session.py status
ros2 run qianli_training_scenarios run_vm_session.py stop
```

## Limits

The current QianLi `ideal_kinematic_sim` imposes commanded body motion and
removes robot contact physics. It does not validate wheel traction, roller
configuration, motor dynamics or real encoder odometry. Evaluation therefore
checks the physical Gazebo trajectory against obstacle polygons separately;
an action success alone is insufficient evidence of collision-free motion.
No machine-learning training algorithm is added in this milestone. This is
the scene/task/evaluation infrastructure for navigation and future training.
