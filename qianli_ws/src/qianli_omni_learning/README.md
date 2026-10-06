# qianli_omni_learning

This module supplies a sensor-only local velocity controller and a reproducible
CPU Cross-Entropy Method (CEM) parameter search. It does not train a neural
network, PPO policy, global planner, SLAM system, or wheel-contact dynamics.
The package belongs to the existing `/home/ros/QianLi/qianli_ws` workspace and
uses the installed `qianli_training_scenarios` geometry.

## Files and interface

- `scripts/policy.py`: `HolonomicPolicy(params).action(goal_body, yaw_error,
  scan_ranges, scan_angles, dt)` returns a NumPy array `[vx, vy, wz]` in the body
  frame. Call `reset()` at episode boundaries. `load_parameters(path)` reads
  `params` from policy JSON.
- `scripts/environment.py`: ideal 2D kinematics, exact oriented-box raycasting,
  octagonal footprint/padded footprint SAT checks from a scene manifest.
- `scripts/train.py`: seeded CEM search, immutable held-out split, real history,
  parameter files and paired offline episode results.
- `scripts/evaluate.py`: evaluation of frozen policy files, with optional
  preferred-speed-matched baseline ablation. This command does not train or
  write policy parameters.
- `scripts/omni_ros_eval.py`: one independent ROS task, with odometry/lidar
  control, Gazebo truth scoring, telemetry watchdog and persistent failure logs.
- `scripts/run_comparison.py`: restart Gazebo for each task and each policy.
- `config/omni_tasks.json`: waypoints, training/evaluation splits, shared limits
  and all acceptance thresholds.
- `policies/`: installed frozen baseline/trained policies, actual training
  history, offline comparisons, ablations and training provenance.

The policy observes only a relative local waypoint, heading error, lidar ranges
and angles, and elapsed control period. The policy has no manifest, obstacle
coordinates or absolute robot position. Pose is used by the evaluation
environment to simulate observations and to score the trajectory. ROS supplies
local goals using odometry and a known episode spawn; ground truth is reserved
for an independent scorer.

Both the fixed baseline and learned policy use exactly the same observation
format and fixed safety filter. The fair command limits are 0.25 m/s translation
and 0.60 rad/s rotation. Baseline preferred speed is 0.24 m/s; learned preferred
speed cannot exceed the same 0.25 m/s cap. The default baseline is a reasonable
goal/repulsion controller, not an intentionally impaired policy.

Eight learned parameters are preferred speed, goal gain, repulsion gain,
repulsion radius, tangent gain, acceleration limit, yaw gain, and preview time.
The fixed safety filter chooses among heading candidates using predicted lidar
endpoints, a 0.06 m padded octagon and a common 0.04 m finite-beam guard. Policy
inputs use 72 uniformly spaced body-frame rays capped at 4 m. A ROS wrapper
should transform the lidar frame, resample consistently, and replace no returns
with 4 m. In the current simulator, lidar XY offset is zero, so taking every
fifth beam from the 360-beam scan supplies the 72 body-frame rays. A sensor with
different XY/yaw mounting requires transforming scan points before this API.
A near-zero goal commands zero translation, supporting actual in-place
rotation without a harness artificially setting linear speed to zero.

## Reproduce

Run these commands in the ROS virtual machine. New results go under a fresh
workspace `log/` directory; installed frozen records are not overwritten.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ros/QianLi/qianli_ws
colcon build --symlink-install
source install/setup.bash

scene_share="$(ros2 pkg prefix --share qianli_training_scenarios)"
learning_share="$(ros2 pkg prefix --share qianli_omni_learning)"
run_dir="$PWD/log/omni_learning_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1

ros2 run qianli_omni_learning train.py \
  --scene-root "$scene_share/generated" \
  --tasks "$learning_share/config/omni_tasks.json" \
  --output "$run_dir/training" \
  --iterations 8 --population 24 --workers 4 --seed 20261005
```

Only `train_000` and `train_001` contribute to CEM updates and checkpoint
selection. `test_101` is opened only after the selected policy has been written;
it never tunes parameters or selects checkpoints. The identical task suite is
used across scene splits so the held-out claim concerns obstacle-layout
generalization, not unseen task instructions. Evaluation checks full box
geometry rather than the sparse policy lidar.

The rollout time step comes from the suite (0.1 s) with two integration samples
per command. Stop tolerances come from the episode configuration. The objective heavily penalizes
failure and padded-footprint intersection, then considers elapsed time, path
length, command variation, low clearance and integrated heading error. An
integral avoids rewarding an artificially extended low-error settling tail.
History includes every sampled parameter vector, score, elite choice, and distribution update.
The fixed baseline participates as the initial training checkpoint so a worse
training policy is never selected merely because it was sampled later.

Evaluate the installed frozen policies, including the speed-matched ablation:

```bash
ros2 run qianli_omni_learning evaluate.py \
  --scene-root "$scene_share/generated" \
  --tasks "$learning_share/config/omni_tasks.json" \
  --baseline "$learning_share/policies/baseline_policy.json" \
  --trained "$learning_share/policies/trained_policy.json" \
  --output "$run_dir/offline_frozen.json" --speed-ablation
```

To evaluate newly trained files, use `$run_dir/training/baseline_policy.json`
and `$run_dir/training/trained_policy.json` instead. Add `--dt 0.05` to test the
offline transfer to the nominal ROS 20 Hz control period. Evaluation never fits
parameters or selects checkpoints.

## Gazebo/ROS comparison

The seven tasks test positive/reverse lateral motion through a 1.20 m doorway,
fixed-heading diagonal pillar bypass, in-place 90/180 degree rotations,
simultaneous translation/rotation and a fixed-heading S route. These prescribed
local routes do not establish autonomous exploration of the entire building.

The current VM uses Xvfb `:98` and software rendering for GPU lidar. Xvfb and
`xdpyinfo` must already be available. Reuse the shell variables defined above:

```bash
if ! xdpyinfo -display :98 >/dev/null 2>&1; then
  Xvfb :98 -screen 0 1600x1000x24 -nolisten tcp \
    > "$run_dir/xvfb.log" 2>&1 &
fi
for attempt in $(seq 1 20); do
  xdpyinfo -display :98 >/dev/null 2>&1 && break
  sleep 0.2
done
xdpyinfo -display :98 >/dev/null

ros2 run qianli_omni_learning run_comparison.py \
  --variant test_101 --output "$run_dir/gazebo_frozen"
```

For the new training outputs:

```bash
ros2 run qianli_omni_learning run_comparison.py \
  --variant test_101 --output "$run_dir/gazebo_retrained" \
  --policy-directory "$run_dir/training"
```

The default comparison runs seven tasks for each of two policies, with 14
independent Gazebo startups. Each starts at its declared known spawn; feedback
uses the spawn transform plus `/odom`. The policy never receives ground truth.
The helper sets `ROS_DOMAIN_ID=79`, `GZ_PARTITION=qianli_omni_eval_v1`,
`DISPLAY=:98` and `LIBGL_ALWAYS_SOFTWARE=1`. It disables Nav2, SLAM, RViz and the
interactive GUI so Nav2 cannot compete for `/cmd_vel`. Control runs nominally
at 20 Hz. Cleanup verifies the identity of this comparison's process groups.

The telemetry watchdog monitors wall-clock age of clock, odometry, truth and
scan. Age over 1.5 s sends zero velocity and waits for recovery. Any stream
remaining older than 12 s fails the episode and preserves its report. Pauses,
failure reasons and invalid scans remain in the record; infrastructure failures
are not silently counted as policy passes. Output includes `comparison.json`
and each episode's `result.json`, `evaluation.log` and `bringup.log`.

Controller waypoint acceptance is 0.12 m and 0.08 rad. Independent truth scoring
requires terminal error <= 0.15 m and 0.12 rad, all waypoints, timeout compliance
and zero raw/padded footprint intersection samples. Applicable specialty gates
also require fixed heading <= 0.10 rad, lateral cross-track <= 0.15 m, rotation
drift <= 0.06 m and actual simultaneous motion >= 0.50 s. All applicable hard
constraints must pass before the configured exponential score is awarded.

O06 heading error refers to the active commanded waypoint heading, matching
the frozen offline evaluator. XY tracking deviation is distance to the reference
polyline. Ground truth motion, not command contents, determines cross-track,
rotation drift and simultaneous motion. The saved `task_spec` and installed
`config/omni_tasks.json` contain the complete scoring contract.

## Interpretation

`offline_comparison.json` contains paired training and held-out results,
including failed episodes and trajectories. A time improvement counts as a
useful improvement only if completion and collision safety do not regress.
Minimum reported clearance is a conservative separating-axis gap for the
padded octagon; it is not exact Euclidean distance. Failures may not be omitted.
This refers to offline `min_padded_sat_margin_m`; ROS reports use the independent
polygon-clearance scorer. These remain sampled geometry checks, not physical
contact-force measurements.

This establishes parameter-learning infrastructure and a synthetic control
baseline. A successful offline or ROS kinematic rollout cannot establish real
omni-wheel friction, motor/encoder direction, actuator response, physical
contact, localization robustness, or performance in a real teaching building.
Those require additional dynamics and real measurements.

## Recorded offline run

Seed `20261005`, 8 generations, 24 candidates per generation, 4 CPU workers.
The fixed baseline plus candidate rollouts produced 2,702 training episodes on
`train_000/train_001`; wall time including final evaluations was 130.688 s.
Full per-candidate history is in `policies/training_history.json`.

The selected parameters were frozen before opening `test_101`. Both policies
passed all seven held-out tasks, with zero raw/padded footprint intersections.
Their total simulated task times were 132.1 s (baseline) and 119.8 s (trained),
a 9.31% reduction. This is an efficiency improvement; success rate was already
100% and did not improve. The identical task instructions across splits mean
this is obstacle-layout evaluation, not proof of unseen-task generalization.

An explanatory ablation kept the baseline gains but matched its preferred speed
to the trained value. It passed 7/7 in 128.1 s; the trained policy still reduced
time by 6.48% relative to that matched-speed baseline. The ablation did not select
or change trained parameters. A separate 0.05 s control-period evaluation also
passed 7/7 for both policies (131.75 vs 119.65 s). Neither result is a ROS/Gazebo
or physical robot measurement; ROS integration results are recorded separately
below.

A reward-review attempt was stopped after two generations, before loading the
held-out map, because averaging heading error could reward a long settling tail.
The original diagnostic is preserved in the delivery under
`evidence/reward_review_attempt/`. It is not part of final policy
selection. The final run uses integrated heading error and is reproducible by
the installed command above. The frozen `policies/trained_policy.json` SHA-256 is:

```text
8587cf88912e87fb19545a9e1d1bd4929150168b9cce7bc4a3ad34807bdd1de8
```

`policies/training_provenance.json` records the original training environment
and script hashes. ROS scoring/watchdog integration was subsequently revised;
current ROS script hashes belong in a separate integration provenance record,
without replacing original training provenance or changing frozen parameters.

## Actual ROS retest results

The first O03 run encountered software-rendering telemetry interruption; its
original logs are retained. The current full retest uses corrected scoring and
watchdog contracts with unchanged frozen policy parameters. The final `completed` comparison and every task's hard constraints were checked before recording the following results.

The full retest completed all 14 independent episodes on `test_101`:

| Policy | Passed | Total simulated time | Raw / padded intersections |
| --- | ---: | ---: | ---: |
| Fixed baseline | 7/7 | 137.06 s | 0 / 0 |
| Frozen CEM policy | 7/7 | 125.01 s | 0 / 0 |

The reduction is 12.05 s (8.79%) on this single paired suite. Both policies
already passed every task, so this shows efficiency improvement rather than
increased success rate. Times include the common 0.7 s stopping interval;
offline times omit that interval. All specialty constraints were independently
rechecked from final JSON metrics. Maximum lateral cross-track was 8.82 mm;
in-place rotation drift was zero in this ideal simulation. O06 simultaneous
translation/rotation lasted 7.54 s baseline and 6.84 s trained, with maximum
active-waypoint yaw errors 0.2191 and 0.2146 rad. Both meet the fixed gates.

Maximum terminal position/yaw error over the 14 episodes was 0.1215 m /
0.0769 rad. Minimum padded polygon clearance was 0.1849 m. All stopping checks
passed. The final suite had zero telemetry pauses and zero empty lidar frames.
There was no tuning of trained parameters using these Gazebo results.

Remote records:
`/home/ros/QianLi/qianli_ws/log/omni_learning_v0_1/gazebo_comparison_verified`.
The first interrupted comparison remains separately at `gazebo_comparison` in
the same log parent. User-facing delivery includes `REPORT.md`, frozen policies,
`final_audit.json`, `integration_provenance.json`, and complete
`evidence/gazebo_verified`, `evidence/gazebo_interrupted`, and
`evidence/reward_review_attempt` folders. These original failures are not merged
into the final 14-episode comparison.
