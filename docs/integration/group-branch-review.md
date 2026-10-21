# Group branch interface review

> Historical review of the revision below. For current lab setup and the
> implemented experiment controller connection, use the
> [connected-session guide](connected-session.md). In particular, the old
> experiment-node TODO finding below does not describe the current group branch.

Inspected published branch: `placing_blocks_pred_motion`, commit
[`301e1233f5b67f439b5bd541d2903863498e0988`](https://github.com/ALMOWAFI/HRI2026-Legibility-Of-Robots-Motion/tree/301e1233f5b67f439b5bd541d2903863498e0988).
Remote heads were checked on 2026-10-19. The older local group checkout contains
uncommitted changes; this work did not overwrite or publish them.

| Published component | Finding |
|---|---|
| `tools/make_layout.py` | Computes an arc and checks IK reachability. `--write` updates `config/experiment.yaml`; it does not publish a persistent pose stream. |
| `tools/place_blocks.py` | Reads the YAML. Preview publishes planned robot trajectories. Its explicitly authorized placement mode guides human placement and calls `set_cubes` after placement. This is a fixed layout procedure, not continuous perception of moved blocks. |
| `src/hri_legible_motion/scene.py` | Applies collision objects through `/apply_planning_scene` and publishes `hri_cubes` markers on `/visualization_marker_array`. The marker publisher is latched but its process exits; it is not a durable layout source for later subscribers. |
| `tools/spawn_cubes.py` | Re-publishes the configured cube markers and collision objects. It does not add a table model. |
| `tools/test_motion.py` | Creates a straight, fixed-orientation Cartesian reach using a minimum-jerk progress profile and IK. It is a reasonable predictable baseline; it is not RRT-Connect or FK-based legibility optimization. |
| `src/hri_legible_motion/legibility.py` | `legible_path` and `legibility_score` remain unimplemented. Study notes propose a fixed lateral bend, a different generator from this optimizer. |
| `src/hri_legible_motion/robot.py` | Uses `/compute_fk`, `/compute_cartesian_path` and `/move_group/display_planned_path`. Planning returns `JointTrajectory`. Its controller operations remain separate and unchanged. |
| `scripts/experiment_node.py` | Documents `/trial_start` with `TrialInfo`, but the published implementation is TODO. A comment is not an active publisher. |
| `msg/TrialInfo.msg` | Provides trial ID, target block ID, condition and bend. The new optional bridge accepts baseline/legible with bend zero. |

The supplied `experiment.yaml` confirms group `arm`, tool `arm_tool0`, frame
`arm_base_link`, fixed start joints and all four block positions. Its SHA256 is
`f18d74acd27d0071b61787947b4ec9c219bdb35828d7a9bcd66a21ad791136ef`.
This hash identifies the original downloaded file; Git may normalize line endings
in the saved `config/experiment.lab.yaml`. The robot description, SRDF,
calibration/mesh files, a real baseline bag and the scene/TF snapshot were not
successfully captured from the lab. No such files have been fabricated.

The group declares MIT in its manifest. No substantial group/third-party source
has been copied into the new ROS integration: it uses the supplied configuration,
original adapter code, ordinary ROS dependencies and the existing optional
message package. This repository has no owner-selected license grant; its catkin
license field is an explicit unspecified-license placeholder, not a relicensing.

Primary API references used to check the integration:

- [MoveIt 1 Python planning/retiming API](https://github.com/moveit/moveit/blob/noetic-devel/moveit_commander/src/moveit_commander/move_group.py)
- [FK service contract](https://github.com/moveit/moveit_msgs/blob/master/srv/GetPositionFK.srv)
- [State validity contract](https://github.com/moveit/moveit_msgs/blob/master/srv/GetStateValidity.srv)
- [Cartesian path contract](https://github.com/moveit/moveit_msgs/blob/master/srv/GetCartesianPath.srv)
