# Planner → optimizer → downstream handoff

The [connected-session guide](connected-session.md) describes the new optional
group experiment mode, common trajectory output and downstream service contract.
Use that mode for the group's `experiment-pipeline` controller; the older
`/trial_start` bridge below is for simpler zero-bend, uniquely numbered producers.

The two nodes are implemented, with a persistent layout publisher and manual trial
sender. This package does not execute robot motion. Tests exercise real numerical
optimization with simulated ROS messages, MoveIt responses and FK. They do not
establish that RRT-Connect or the calibrated UR5e works in the lab installation.

## Exact topic contract

All custom messages belong to `hri_legibility_preview`. Topics below are absolute
ROS names; remap both ends together if your deployment uses different names.

| Topic | Type | Publisher → subscriber |
|---|---|---|
| `/hri/block_layout` | `BlockLayout` | Persistent YAML layout publisher → planner |
| `/legibility/trial_request` | `TrialRequest` | Manual sender or experiment adapter → planner |
| `/trial_start` | `hri_legible_motion/TrialInfo` | Optional existing group interface → planner; enable explicitly |
| `/legibility/baseline` | `BaselinePlan` | Planner → optimizer and downstream baseline consumer |
| `/legibility/optimized_candidate` | `OptimizedCandidate` | Optimizer → teammate's validation/retiming node |
| `/legibility/trajectory_proposal` | `TrajectoryProposal` | Final numerical output for either condition → session / downstream validator |
| `/legibility/motion_feedback` | `MotionFeedback` | External executor (not supplied) → optional group session |
| `/legibility/status` | `std_msgs/String` containing JSON | Both nodes → operator/logger |
| `/legibility_preview/planner/display` | `moveit_msgs/DisplayTrajectory` | Planner → RViz |

The layout is latched and its publisher remains alive. Trajectories and trials
are **not latched**. Start consumers/recording before sending trials. ROS topics
are not durable job queues: inspect status for each trial, and use a new unique
trial ID for retries after a failed accepted job. A busy node reports rejection;
it does not maintain a pending trial queue. Duplicate IDs are rejected within
each node's lifetime. Downstream must handle duplicates/restarts independently.

`BlockLayout` carries a frame, block IDs/names/poses, the desired tool hover Z, and
a SHA256 layout ID. `legibility_ros.layout.layout_digest` defines the digest over
canonical JSON (header timestamp excluded). The planner validates the whole
layout and snapshots it when accepting a trial. A layout update never triggers
planning. Invalid replacement layouts invalidate the previous layout until a
valid one arrives. An in-flight trial keeps its original snapshot.

`TrialRequest` identifies a target ID, unique trial ID, and condition (`baseline`
or `legible`). A nonempty layout ID must match the received layout; an empty ID
binds the trial to the latest valid received layout. A target ID is mapped to its
row by ID, never used directly as an array index. The optional group bridge uses
the existing `TrialInfo` type without copying it. It rejects nonzero `bend`, since
that field does not parameterize this optimizer. Use the optimizer's cost/deviation
parameters instead. Group trial IDs are converted to strings; use a single trial
source per session and restart for a new session with reused numeric IDs.

`BaselinePlan` includes the trial/condition, complete layout snapshot, robot
model/group/tool identifiers, config digest, baseline mode, full start state and
`RobotTrajectory`. Legible trials proceed to optimization. Baseline trials bypass
optimization and do not produce a misleading "optimized" candidate.

## Teammate's downstream interface

Subscribe to `/legibility/trajectory_proposal` with `TrajectoryProposal` for
both conditions. The separate `/legibility/optimized_candidate` topic retains
the detailed legible-result message. The `source` field contains the entire `BaselinePlan`; `trajectory`
contains candidate positions on the original nominal timestamp grid. Velocity,
acceleration and effort arrays are cleared. Metrics report solver success,
numerical acceptance, before/after legibility and nominal joint motion cost.

The publication contract is:

- Only numerically accepted results are published; failures appear on the status topic.
- `retiming_required=true`, `physical_validation_required=true`, `execution_ready=false`.
- End joint configurations, joint order and nominal timestamps match the baseline exactly.
- All candidate goals and the intended ID travel with the trajectory.
- A legible trial must not silently fall back to the baseline after optimizer rejection.

For baseline trials, the proposal has `optimization_applied=false`. The baseline
topic remains diagnostic/intermediate: messages with `condition == "legible"`
are optimization inputs and must not trigger execution.

The teammate owns retiming, validation against actual measurements/model/scene,
controller interpolation/dynamic checks, current-start-state checks, final
score/cost evaluation after retiming, and explicitly authorized execution of the
exact validated trajectory. If downstream modifies positions or timestamps, it
must repeat the affected checks. The historical `robot_interfaces.py` execution
and full-validation stubs still raise explicitly; no new execution implementation
is provided here. Sample checks in the optional preview path are not a physical
validity certificate.

If ground-truth measurements change block goals or their coordinate frame, feed
those into a valid **input layout before planning/optimization**. A downstream
validator can compare a candidate against measurements; silently moving its goal
after optimization invalidates the original observer score and endpoint contract.

## Frames, robot data and planning modes

The supplied lab snapshot identifies `cu_ur5e`, group `arm`, link `arm_tool0`, and
goal frame `arm_base_link`. MoveIt reported planning frame `base`. FK explicitly
requests the layout frame; labels are never substituted for transforms. A layout
in a different frame is rejected until transformed by its provider. The actual
model and bounds come from the loaded MoveIt/URDF, not generic UR5 kinematics.

The tool goal is `[block.x, block.y, touch_z + hover_height]`: Z is **0.053 m** in
the supplied config. Cube-center Z is **−0.074 m**; setup pointing Z is **0.033 m**.
These are different physical quantities. Tool orientation comes from FK at the
configured start pose. The initial arm configuration is the configured study
start, merged into a full model state snapshot; planning does not move the robot
to that start. The downstream executor must compare the actual state to it.

The topic pipeline defaults to `rrtconnect`. The planner looks up the loaded
group's enabled planner IDs and accepts only definitions whose type is
`geometric::RRTConnect`. It auto-selects when exactly one exists, otherwise an
explicit `planner_id` is required. It sets the planning pipeline (default `ompl`)
and MoveIt namespace (explicit `/`, independent of our node namespace). Missing
configuration or unsuccessful MoveIt results fail, without a planner fallback.

`baseline_mode:=cartesian` instead uses collision-aware Cartesian IK to create
the group's straight-reach geometry with fixed start orientation. It is not
RRT-Connect. Both modes retime the baseline with MoveIt before handing it over.
Consequently the resulting duration is not guaranteed to equal the group's
nominal four seconds. Do not describe the retimed preview as an identical copy
of the group's tested timing. Agree on the study timing policy before participant
trials, since duration/speed changes can affect the comparison.

Numerical optimization uses up to 11 support waypoints by default, interpolates
joint **offsets** onto every original baseline waypoint, then recomputes bounds,
deviation, budget and score checks on the full grid. It does not replace the dense
baseline with a coarse path. This is a surrogate solve, not proof of a full-grid
optimum. FK has an exact, per-request bounded cache and a 30,000-call budget.
Non-improving or infeasible results may be rejected; no improvement is guaranteed.
The observer measures model legibility, not demonstrated human recognition.

## Build on the ROS 1 computer

The group README mentions Noetic, but the supplied terminal paths used
`/opt/ros/one`. The installed version has not been captured. Use the already
sourced lab ROS/MoveIt environment, not an assumed distro-specific setup path.
This integration targets the ROS 1 Python 3 `moveit_commander` API with the
four-element `plan()` result and the retiming API documented by MoveIt 1.

From a separate workspace (after sourcing the existing robot workspace):

```bash
mkdir -p "$HOME/legibility_preview_ws/src"
cd "$HOME/legibility_preview_ws/src"
git clone --branch integration/preview-planner-optimizer https://github.com/ALMOWAFI/The-legibilityOptimizer-HRI2026.git hri_legibility_preview
cd "$HOME/legibility_preview_ws"
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
```

Dependencies are declared in `package.xml`: ROS messages, rospy, MoveIt commander,
NumPy, SciPy and PyYAML. The optional `/trial_start` bridge additionally requires
the group's built and sourced `hri_legible_motion` message package. It is not
required for the standalone manual trial sender.

## Launch and inspect (no execution)

Keep the existing MoveIt bringup running. Read its actual planner settings:

```bash
rosversion -d
rosversion moveit_commander
rosparam get /move_group/planner_configs
rosparam get /move_group/arm/planner_configs
rosservice type /compute_fk
```

Launch using the active group config file:

```bash
roslaunch hri_legibility_preview pipeline.launch experiment_config:=/absolute/path/to/group/config/experiment.yaml
```

An omitted planner ID auto-selects a unique configured RRT-Connect planner. Override
`planner_id`, `planning_pipeline_id`, `planner_configs_param`, `group_planners_param`
and `moveit_ns` if the inspected lab configuration uses other names/namespaces.
The checked-in `config/experiment.lab.yaml` is a user-supplied snapshot, used only
if no config path is given. Restart the pipeline after editing its config file.

In another sourced terminal, watch status and/or record the handoff before a trial:

```bash
rostopic echo /legibility/status
# Or in another terminal; Ctrl+C finishes the recording:
rosbag record -O legibility_handoff.bag /hri/block_layout /legibility/trial_request /legibility/baseline /legibility/optimized_candidate /legibility/trajectory_proposal /legibility/status
```

Send one trial (IDs in the lab config are 0=orange, 1=red, 2=yellow, 3=green):

```bash
rosrun hri_legibility_preview send_trial.py --trial-id preview-001 --target-id 0 --condition legible
```

Use `--condition baseline` for a predictable trial, with a new ID. To consume the
team's existing `/trial_start`, launch with `subscribe_group_trials:=true`; the
group package must be built/sourced and its experiment node must actually publish
that message. No changes to `place_blocks.py` are needed.

Inspect connections with `rostopic info /legibility/baseline` and
`rostopic info /legibility/trajectory_proposal`. Set RViz's planned-path topic to
`/legibility_preview/planner/display` for baseline display. The optimizer's output
is a custom candidate message, not an execution or display command.

The older `~plan` and `~optimize` services remain available. Their
`preview_validation` request flag enables optional MoveIt retiming and discrete
checks through `/check_state_validity`; false leaves downstream validation to the
teammate. Service calls return results directly; they do not publish topic jobs.

## Offline verification

```bash
python -m pip install -r requirements-dev.txt
python -m unittest discover -s tests/experimental -v
python -m unittest discover -s tests/ros1 -v
python -m experimental.legibility_optimizer.demo --output build/legibility-demo
python docs/research/build_manuscript.py
```

The topic tests exercise the actual callbacks and numeric optimizer with an
in-memory ROS bus and simulated MoveIt/FK. They cover late layout subscribers,
RRT configuration/failed planning, target mapping, immutable in-flight layouts,
busy/duplicate trials, baseline bypass, rejected optimizations, output endpoint
integrity and publication. See [verification](verification.md) for actual results
and [changes](changes.md) for the review summary.
