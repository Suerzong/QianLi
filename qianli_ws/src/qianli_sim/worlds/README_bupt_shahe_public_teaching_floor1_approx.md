# Approximate Gazebo world

`qianli_bupt_shahe_public_teaching_floor1_approx.sdf` is the first 3D geometry
test generated from the supplied photographed plan.

- World extent: approximately `44.4 m × 35.9 m` from the provisional 1.20 m door calibration.
- Floor: one static box.
- Walls: long horizontal/vertical/45-degree dark strokes selected by a Hough pass,
  represented as static boxes, height `2.8 m`, thickness `0.14 m`.
- It is not the default QianLi world and is not a surveyed architectural model.
  The extraction can include furniture, text, duplicate wall fragments, and
  non-wall drawing strokes. Door openings are not semantically reconstructed.

Use it explicitly after sourcing the workspace:

```bash
ros2 launch qianli_sim sim.launch.py \
  world:=$(ros2 pkg prefix qianli_sim)/share/qianli_sim/worlds/qianli_bupt_shahe_public_teaching_floor1_approx.sdf \
  gui:=true rviz:=false
```

The resulting world is useful for checking basic Gazebo loading, laser returns,
and whether the QianLi footprint can move through the rough layout. Replace it
with a manually traced wall/door model before treating paths as building-valid.
