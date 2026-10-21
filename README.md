# Planner and optimizer

The contribution to the team pipeline is two ROS 1 nodes:

| File | What it does |
|---|---|
| [scripts/planner_node.py](scripts/planner_node.py) | Receives block layout and trial requests, plans with MoveIt RRT-Connect, and publishes the baseline trajectory. The planning code is in `plan_baseline()`. |
| [scripts/optimizer_node.py](scripts/optimizer_node.py) | Receives the baseline, runs FK-based legibility optimization, and publishes an accepted trajectory proposal. The optimization callback is `handle_optimize()`. |
| [launch/pipeline.launch](launch/pipeline.launch) | Starts both nodes and the existing YAML block-layout publisher. |

These node files contain the actual callbacks, topic wiring and startup code.
The standalone [optimizer mathematics](experimental/legibility_optimizer/optimizer_core.py)
remain reusable without ROS. Shared robot access and configuration helpers are
in `legibility_ros/`.

```text
Block layout + trial request
        -> planner_node.py
        -> /legibility/baseline
        -> optimizer_node.py
        -> /legibility/trajectory_proposal
        -> team's separate validation and execution stage
```

The nodes never execute motion. For `baseline` trials the optimizer passes the
baseline through; for `legible` trials it publishes only an accepted optimization.
Both outputs require downstream retiming and physical validation.

## Run the nodes

After [building the packages](docs/integration/connected-session.md#build-and-first-lab-checks),
source the workspace's `devel/setup.bash` in each terminal. With the lab MoveIt
stack running, use the team's current measured configuration:

```bash
roslaunch hri_legibility_preview pipeline.launch \
  experiment_config:="$(rospack find hri_legible_motion)/config/experiment.yaml"
```

This launch starts the layout publisher, planner and optimizer. Game, interface
and execution are separate team components.

## Check the data flow

In another sourced terminal, run these one at a time:

```bash
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition baseline
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition legible
```

Each command sends one trial and waits for its matching output. `PASS` means
that trial crossed the pipeline; a missing connection, rejection or timeout
produces `FAIL`. Neither command requests execution. Use `--timeout 300` if
the real FK service needs more time.

| Topic | Publisher -> subscriber |
|---|---|
| `/hri/block_layout` | Layout publisher -> planner; retained for new subscribers |
| `/legibility/trial_request` | Team experiment or check tool -> planner |
| `/legibility/baseline` | Planner -> optimizer |
| `/legibility/trajectory_proposal` | Optimizer -> team's downstream stage |
| `/legibility/status` | Both nodes -> diagnostics |
| `/legibility_preview/planner/display` | Planner -> RViz |

Trajectory topics publish when a trial is processed; they are quiet while idle.
The exact message definitions and handoff requirements are in the
[topic contract](docs/integration/README.md).

## Offline tests

```bash
python -m pip install -r requirements-dev.txt
python -m unittest discover -s tests/experimental -v
python -m unittest discover -s tests/ros1 -v
```

The tests use the real numerical solver with substituted ROS/MoveIt boundaries.
Live ROS and robot operation still need lab verification.

Existing optional [team-session integration](docs/integration/connected-session.md)
and [downstream execution](docs/integration/execution.md) have their own guides.
