# UR5e connected experiment

Run the nodes, send a trial, check the output. These commands do not move the robot.

Use branch `integration/preview-planner-optimizer` with the group's
`integration/rrt-optimizer-session` branch. First-time setup:
[build both packages](docs/integration/connected-session.md#build-and-first-lab-checks).
In each terminal, source your ROS workspace's `devel/setup.bash`.

## 1. Start the nodes

With the lab's MoveIt stack already running:

```bash
roslaunch hri_legibility_preview group_pipeline.launch \
  experiment_config:="$(rospack find hri_legible_motion)/config/experiment.yaml" \
  display:=false
```

This starts:

- `/legibility_preview/block_layout` — publishes block poses from the shared YAML.
- `/legibility_preview/planner` — requests a MoveIt RRT-Connect trajectory.
- `/legibility_preview/optimizer` — optimizes legible trials and publishes the result.
- `/legibility/executor` — provides retiming/validation and motion services, with execution disabled by default.

The group participant display starts with `display:=true`. Run the interactive
game/session separately after the lab checks below. Use `downstream:=false` if
you only need planning/optimization without the downstream node.

## 2. Check that data goes through

In another sourced terminal:

```bash
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition legible
```

It checks the registered publishers/subscribers and message types, waits for
transport connections and the block layout, then sends one uniquely identified
trial. It must receive the matching baseline, accepted optimizer result and
final proposal before printing:

```text
PASS: matching trial data crossed the pipeline; no execution was requested
```

A missing connection, rejected plan/optimization, wrong output or timeout prints
`FAIL` and exits with code 1. It never calls a motion/execution service.
Use `--timeout 300` if a real FK optimization needs more than the default 180 seconds.

To test the baseline without optimization:

```bash
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition baseline
```

## Topic connections

| Topic | Publisher → subscriber | Data arrives |
|---|---|---|
| `/hri/block_layout` | Layout node → planner | Once at startup, retained for new subscribers |
| `/legibility/trial_request` | Check tool or experiment → planner | One message per requested trial |
| `/legibility/baseline` | Planner → optimizer | When planning succeeds |
| `/legibility/optimized_candidate` | Optimizer → check/diagnostics | When a legible optimization is accepted |
| `/legibility/trajectory_proposal` | Optimizer → downstream validation stage | Final output for either accepted condition |
| `/legibility/validated_trajectory` | Downstream node → preview/logging | After requested validation passes |
| `/legibility/motion_feedback` | Executor → game/session | Measured onset/completion or failure of a confirmed command |
| `/legibility/status` | Planner + optimizer → terminal/check tool | Progress or rejection |

**Trajectory topics are quiet while idle.** They are triggered by trials, not a
continuous stream. The robot's `/joint_states` is the live state stream.

For direct inspection:

```bash
rostopic info /legibility/baseline
rostopic echo /legibility/status
# Another terminal, started before a trial:
rostopic echo /legibility/trajectory_proposal
# Check the robot's live state stream:
rostopic hz /joint_states
```

Use only this launch for the connected pipeline. Do not also start the group's
legacy `hri.launch` or `motion_node.py`.

## 3. Check the downstream stage without motion

Capture the lab's model, collision objects and controller information:

```bash
rosrun hri_legibility_preview inspect_execution.py
```

Set the measured environment collision-object IDs in `config/execution.yaml`.
The group tools add the cubes; they do not add a table. With the real scene ready:

```bash
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition baseline --validate
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition legible --validate
```

These commands never request execution. They reject absent geometry, invalid
paths, failed retiming or a changed scene instead of reporting success.

The software now includes block poses → planner → optimizer → validation/executor
→ measured feedback → game/display. **Live ROS and robot operation are still
unverified.** Follow the [execution lab guide](docs/integration/execution.md) to
check the calibrated model/controller and enable operator-confirmed motion.
