# Connected experiment session

The group controller can now use the standalone RRT/FK pipeline through its
optional `--pipeline` mode. The companion group branch is
`integration/rrt-optimizer-session`, based on `experiment-pipeline` at
`1bf40badfaad63932fee37e0a24fde4b2a0e2eee`.
The optimizer branch is `integration/preview-planner-optimizer`.

```text
same experiment.yaml
  ├─ block-layout publisher → /hri/block_layout
  └─ group experiment controller (--pipeline)
       → /legibility/trial_request
       → RRT-Connect planner → /legibility/baseline
       → optimizer (baseline bypass OR accepted FK optimization)
       → /legibility/trajectory_proposal
       → downstream retiming / configured robot and scene checks
       → operator types go
       → separately enabled executor
       → /legibility/motion_feedback → experiment scoring / logs
```

The new launch file starts layout, planner, optimizer, the downstream node with
execution disabled, and optionally the existing participant display. It does **not** start the group's placeholder replay
`motion_node.py`. The normal group mode remains available independently. Never
start the two motion/session modes together. The legacy `/trial_start` bridge is
disabled in the new launch; the session does not publish `/trial_start` or
`/motion_home`, or listen to uncorrelated `/motion_status` strings.

## What is reused

The group keeps its scheduling, counterbalancing, button mapping, participant
display, ratings and CSV logger. `experiment_node.py` adds an opt-in transport
adapter and `--config`; its cache/replay mode remains the default.
Block placement and robot/controller code are unchanged.

In pipeline mode the experiment label `predictable` maps explicitly to `baseline`.
That baseline is **RRT-Connect**, not a guarantee of a straight Cartesian reach.
The experimental conditions must be described accordingly. The old `bend` and
`bend_side` settings do not control this optimizer; joint deviation and cost
parameters do. No hand-designed bend is introduced.

This mode replans live, so stochastic RRT results can differ across trials.
Recording exact paths does not make them identical across participants. Agree
on repeatability and matched baseline policy before collecting study data.

Each preparation attempt gets a session UUID plus scheduled trial number plus
attempt UUID. A result must match the target, condition, layout, frame, group,
tool and full YAML digest. The session rejects changed layouts until all nodes
are restarted with one matching config. Preparation failure never substitutes a
baseline for a legible trial.

## One handoff for both conditions

`/legibility/trajectory_proposal` has type
`hri_legibility_preview/TrajectoryProposal`. It carries:

- `source`: complete baseline, start state, layout and trial/model/config metadata.
- `trajectory`: unchanged baseline or accepted optimized candidate.
- `optimization_applied`: false for baseline, true for legible.
- `diagnostics`: bypass information or numerical optimizer diagnostics.
- `retiming_required=true`, `physical_validation_required=true`,
  `execution_ready=false`, for both conditions.

The baseline and optimized-candidate topics remain available for diagnostics.
Downstream should use the proposal as its single final input and must not execute
the intermediate baseline of a legible trial. Proposals are not latched; start
consumers before requesting trials. ROS transport is not a durable queue.

## Downstream interfaces

`legibility_execution` now implements these services in a separate node. The
[execution lab guide](execution.md) gives the checks, commissioning fields and
commands. The session fails when a service is absent, rejects a request, or does
not answer within its timeout. Numerical success alone never authorizes motion.

| Endpoint | Type | Required provider behavior |
|---|---|---|
| `/legibility/validate_trajectory` | `ValidateTrajectory` service | No motion. Retime, validate, and store the exact approved path; return a fresh single-use `validation_id` and duration, or reject. |
| `/legibility/request_motion` | `RequestMotion` service | Accept/reject a separately authorized `home` or `run` command promptly. Enforce command/token deduplication. A `run` needs the exact stored validation token. |
| `/legibility/motion_feedback` | `MotionFeedback` topic | Actual robot events with matching command, trial and validation IDs and measured ROS timestamps. Never synthesize onset from a planning result or controller request. |
| `/legibility/validated_trajectory` | `ValidatedTrajectory` topic | Exact retimed trajectory and validation/context IDs for preview and recording; not an execution command. |

The validator binds its token to the full proposal, loaded model,
joint order, frames, tool, measurements, scene/attachments, timing and final
trajectory. It checks sampled collisions over quintic interpolation, joint and
velocity/acceleration constraints, TCP workspace/orientation/speed, goal and
start tolerances, and the knot-based cost/deviation/observer metrics affected by retiming.
The requested study duration must be achieved safely or rejected; the session
does not force a four-second path by rescaling unchecked timestamps. Discrete
collision and numerical TCP checks are not continuous physical certification.

The executor independently verifies token freshness, current start state,
configuration/model/scene compatibility and controller availability immediately
before dispatch. It sends the exact stored validated trajectory, rejects
expired/used tokens and concurrent commands, and requests cancellation on faults.
Controller behavior and cancellation still require testing on the real stack.
A `home` request reverses only the last completed validated curve in an unchanged
scene, after separate operator authorization; arbitrary direct moves reject.
Do not infer robot validity from the config hash alone.

For a reach, feedback order is `executing → done` or `failed`. `executing` is
measured **arm** motion onset; `done` means verified successful reach completion.
A home command ends with `home` or `failed`. Feedback identifies the command and
its trial/token; old attempts, duplicates, missing onset and backwards timestamps
cannot advance the session. Use the same ROS clock as participant choices.
The provider must publish truthful event timestamps and fault/status details.

Preparation and validation happen before the participant's get-ready phase.
An operator types `go` before each reach request. After a completed trial is
logged, returning home is a separate prompted action. Missing onset, failure or
timeout stops the session rather than automatically sending another motion.
A service timeout does not cancel a remote command: inspect its actual state
before restarting.

Scoring uses measured onset-to-completion duration. The fixed requested duration
remains in session metadata; retain actual duration and controller diagnostics
when assessing condition comparability. `pipeline_attempts.jsonl` binds scheduled
trial numbers/retries to unique pipeline IDs, validation IDs, config/layout IDs,
timestamps and outcome. Exact proposals are saved as ROS-serialized `.proposal`
files and pipeline topics are added to the session bag. CSV schemas remain
unchanged; join repeated scheduled trials through the attempt log and timestamps.

## Build and first lab checks

Use a separate ROS 1 workspace with both branches. Source the **actual installed**
ROS/MoveIt environment first; the lab distribution has not been confirmed.
These commands build the proposed integration without touching the team's
existing checkout:

```bash
mkdir -p "$HOME/legibility_connected_ws/src"
cd "$HOME/legibility_connected_ws/src"
git clone --branch integration/preview-planner-optimizer \
  https://github.com/ALMOWAFI/The-legibilityOptimizer-HRI2026.git hri_legibility_preview
git clone --branch integration/rrt-optimizer-session \
  https://github.com/ALMOWAFI/HRI2026-Legibility-Of-Robots-Motion.git hri_legible_motion
cd "$HOME/legibility_connected_ws"
rosdep install --from-paths src --ignore-src -y
catkin_make
source devel/setup.bash
```

Use the team's current measured YAML for **all** processes. With MoveIt running:

```bash
export HRI_CONFIG="$(rospack find hri_legible_motion)/config/experiment.yaml"
roslaunch hri_legibility_preview group_pipeline.launch \
  experiment_config:="$HRI_CONFIG" display:=false
```

In another sourced terminal, run the planning-only connection and data check:

```bash
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition baseline
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition legible
```

The check subscribes before publishing a unique trial. It verifies ROS master
registrations/types, transport connections, a received layout, and matching
baseline/candidate/proposal messages with preserved endpoints. It prints `PASS`
only after receiving the required data; failure or timeout exits nonzero.
Numerical rejection is a failed trial even when transport is connected. These
commands request no motion and need no downstream executor. `--timeout 300`
allows more time for FK optimization. Run one check at a time.

The layout is static and latched. Trajectories are published per trial, not at a
continuous frequency. To inspect progress separately, use
`rostopic echo /legibility/status`; the live robot state is `/joint_states`.
The graph lookup uses the standard
[ROS 1 Master API](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosgraph/src/rosgraph/masterapi.py).

Only after completing the [execution lab checks](execution.md), including the
measured scene and installed controller checks, enable execution in the launch:

```bash
# Launch group_pipeline.launch with display:=true enable_execution:=true,
# using the reviewed execution_config, then a separate interactive terminal:
export HRI_CONFIG="$(rospack find hri_legible_motion)/config/experiment.yaml"
rosrun hri_legible_motion experiment_node.py P07 --pipeline --config "$HRI_CONFIG"
```

The interactive session requires an operator. Missing commissioning data,
services, validation, confirmation or measured feedback fails explicitly.

## Verification and limits

Offline commands:

```bash
# Optimizer repository:
python -m unittest discover -s tests/experimental -v
python -m unittest discover -s tests/ros1 -v
python -m unittest discover -s tests/execution -v
# Group repository:
PYTHONPATH=src python -m unittest discover -s test -v
```

Tests exercise the real SLSQP solver through a fake ROS/MoveIt/FK transport, plus
group-session preparation, rejection, timing, logging and isolation from the
legacy trigger. They do not establish live ROS connectivity, catkin message
generation, RRT planning on the actual model, physical validity or human legibility.
Those remain lab validation tasks. No third-party implementation was copied
into the optimizer repository; the existing group module is modified in its own
repository.

No live ROS 1/catkin or hardware validation was performed on this development
computer. See the pull request for the recorded offline test results.
