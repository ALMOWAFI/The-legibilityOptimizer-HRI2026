# Downstream node: lab use

`/legibility/executor` implements the missing downstream services. It consumes
the optimizer's exact proposal, retimes it with MoveIt IPTP, checks the result,
stores a single-use validation ID, and accepts a separately confirmed motion
request. Neither a topic message nor a successful optimization moves the arm.

## Build and start

Use the two packages from the [build guide](connected-session.md#build-and-first-lab-checks).
Rebuild **both** catkin packages and source the resulting workspace: the
`RequestMotion` service has new confirmation and dispatch-deadline fields.
Do not run clients generated from the old service definition.

With the lab MoveIt/robot stack already running:

```bash
export HRI_CONFIG="$(rospack find hri_legible_motion)/config/experiment.yaml"
export HRI_EXEC="$(rospack find hri_legibility_preview)/config/execution.yaml"
roslaunch hri_legibility_preview group_pipeline.launch \
  experiment_config:="$HRI_CONFIG" execution_config:="$HRI_EXEC" display:=true
```

This starts layout, planner, optimizer, downstream node and participant display.
Execution defaults to **false**. It does not start the old `motion_node.py`,
`hri.launch`, a controller, or a robot driver. Do not run those legacy experiment
nodes alongside this pipeline.

## Read-only lab checks

```bash
rosrun hri_legibility_preview inspect_execution.py
rosservice type /legibility/validate_trajectory
rosservice type /legibility/request_motion
rostopic info /legibility/trajectory_proposal
rostopic hz /joint_states
rostopic hz /speed_scaling_factor
```

The inspector prints the loaded model/limits fingerprint, full world collision
objects, and controller names/types/resources. It sends no motion commands.
If the controller manager or scene service is namespaced, use its `--controller-manager`
or `--scene-service` option and update the matching execution YAML setting.

Before validation can pass:

1. Use the team's **measured** `experiment.yaml` for every process. The layout
   publisher reports these fixed coordinates; it does not track displaced cubes.
2. Add/verify the measured table and surrounding obstacles in MoveIt. The group's
   `spawn_cubes.py` only adds `cube_B1` through `cube_B4`. Put the actual table/
   environment collision IDs in `required_world_objects`; an empty list rejects.
   No table dimensions or collision geometry are invented by this package.
3. Verify the installed calibrated URDF/SRDF, tool collision geometry, attachments,
   collision matrix, joint limits, and frame transforms. The configured tool is
   `arm_tool0`, group `arm`, goal frame `arm_base_link`; these are distinct from
   the robot's planning frame `base`. Cube geometry/poses are checked after TF
   transformation into the goal frame.
4. The arm must be stationary at `motion.start_joints`. If it is elsewhere, use
   the lab's existing checked MoveIt workflow to reach that pose. This node does
   not invent a direct move from an arbitrary configuration.
5. Check live arm positions **and velocities**, positive increasing ROS timestamps,
   and fresh speed scaling at 1.0. Missing/stale data rejects. Other moving joints
   cannot supply the arm's onset signal.

Then, one command at a time:

```bash
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition baseline --validate
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition legible --validate
# Observe the exact retimed path and check report in another terminal:
rostopic echo /legibility/validated_trajectory
```

Repeat for target IDs 0–3. Failed validation does not send a fallback trajectory.
`--timeout 300` permits a longer read-only preparation wait. FK validation uses
the actual `/compute_fk` service and may be substantially slower than offline tests.

## Enable only after commissioning on the real stack

The supported controller type is
`position_controllers/ScaledJointTrajectoryController`, with exactly the arm's
joint resources and the action named in `experiment.yaml`. Other types reject.
Verify the installed controller uses position/velocity/acceleration quintic
interpolation, including its first-point splice, tracking tolerances, stop and
cancellation behavior. Check that the scene includes the real tool and that
the path has adequate clearance for tracking error. Set
`controller_interpolation_verified: true` only after this check. Set
`commissioned_model_sha256` to the inspector's fingerprint after verifying the
model/limits; the hash identifies the configuration and is not proof of safety.
Restart the nodes after changing YAML files.

At the robot, after these checks and the team's motion review:

```bash
# Restart the same launch with this explicit option:
roslaunch hri_legibility_preview group_pipeline.launch \
  experiment_config:="$HRI_CONFIG" execution_config:="$HRI_EXEC" \
  display:=true enable_execution:=true

# In a separate, sourced interactive terminal:
rosrun hri_legible_motion experiment_node.py P07 --pipeline --config "$HRI_CONFIG"
```

Enabling the node does not move the robot. The session requires the operator to
type `go` for each reach and return. `RequestMotion` carries that confirmation
and a short dispatch deadline. Unknown/consumed/expired validation IDs, mismatched
trials, concurrent commands, or changed context reject. Do not publish motion
requests from other tools; ROS 1 service fields are not authentication.

A successful reach remains at its endpoint. A separately authorized home request
reverses the **last completed, validated spline**, slowed to at least 8 seconds,
only in the same model/scene context. Exact reversal preserves the checked
geometry and slowdown reduces velocity/acceleration. If already at study start,
home only verifies the measurement and sends no goal. Unknown starting poses,
changed environments and failed reaches require the lab's checked recovery workflow.

## What the checks mean

- Joint position/velocity/acceleration extrema are evaluated on the controller's
  quintic polynomial, including between-knot overshoot. Retiming preserves all
  supplied joint knots/endpoints. Uniform duration scaling is followed by fresh
  checks; forcing the study duration can reject a path that is too fast.
- FK workspace, orientation, TCP velocity and whole-robot collision/state validity
  are sampled at at most 20 ms and 0.01 rad per joint by default. TCP velocity uses
  a central finite-difference FK Jacobian. These are discrete numerical checks,
  **not continuous collision certification**, tracking-error envelopes, or proof
  that the measured lab matches its model. Retimed legibility/cost/deviation checks
  use the optimizer's knot-based definitions, not a human-subject result.
- A token binds the exact stored path, model/limits, scene/attachments/ACM/padding,
  goal transform, controller, layout and configuration. Preflight checks these
  again and requires fresh stationary start measurements. The collision checks
  assume a static workcell. Geometry/attachment/ACM updates during motion request
  cancellation; periodic repeated scene messages can conservatively cause a stop.
- `executing` is the timestamp of the first measured arm velocity above 0.01 rad/s.
  `done` requires action success plus endpoint position and arm velocity at rest
  for 0.1 s; its timestamp is the first sample of that verified settling interval.
  Onset/settling thresholds and timestamp alignment need lab characterization.
- Missing onset, stale feedback, changed scaling, changed geometry, failed action,
  or timeout requests cancellation of this node's own goal and latches a fault.
  Cancellation is **requested, not claimed acknowledged**. Inspect the robot and
  controller before restarting; there is no retry, automatic home, dashboard
  recovery, or controller switching. The physical e-stop remains an operator control.

The group records validated trajectories and measured feedback in its session bag
when `record_bag` is enabled (the default). The token/trial IDs connect these to
`pipeline_attempts.jsonl` and the existing trial CSV. Keep recording enabled to
retain the exact executed timing as well as the original proposal.

## Verification

```bash
python -m unittest discover -s tests/experimental -v
python -m unittest discover -s tests/ros1 -v
python -m unittest discover -s tests/execution -v
# In the group repository:
PYTHONPATH=src python -m unittest discover -s test -v
```

These tests use fake ROS/controller boundaries and real numerical calculations.
They do not establish live catkin compatibility, calibrated FK/collisions, or
hardware readiness. No robot commands were run on the development computer.

API references, inspected without copying implementation:
[ROS controller interpolation](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/trajectory_interface/quintic_spline_segment.h),
[UR scaled controller](https://github.com/UniversalRobots/Universal_Robots_ROS_scaled_controllers/blob/main/scaled_joint_trajectory_controller/src/scaled_joint_trajectory_controller.cpp),
[MoveIt Python bindings](https://github.com/ros-planning/moveit/blob/noetic-devel/moveit_ros/planning_interface/move_group_interface/src/wrap_python_move_group.cpp).
The quintic code independently solves boundary equations; no third-party code was copied.
