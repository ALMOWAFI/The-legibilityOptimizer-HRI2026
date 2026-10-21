# UR5e planner and legibility optimizer

**Your contribution to the team is these two nodes. Start with their code:**

| Open this file | Purpose |
|---|---|
| [planner_node.py](https://github.com/ALMOWAFI/The-legibilityOptimizer-HRI2026/blob/integration/preview-planner-optimizer/scripts/planner_node.py) | Receives block poses and a trial request, plans with MoveIt RRT-Connect, and publishes the baseline. |
| [optimizer_node.py](https://github.com/ALMOWAFI/The-legibilityOptimizer-HRI2026/blob/integration/preview-planner-optimizer/scripts/optimizer_node.py) | Receives the baseline, runs FK-based legibility optimization, and publishes the trajectory proposal. |
| [pipeline.launch](https://github.com/ALMOWAFI/The-legibilityOptimizer-HRI2026/blob/integration/preview-planner-optimizer/launch/pipeline.launch) | Starts both nodes and the YAML block-pose publisher. |

```text
Block poses + trial request -> planner -> optimizer -> team's validation/execution
```

The links above open the actual node implementations. The ROS package currently
lives on the [active code branch](https://github.com/ALMOWAFI/The-legibilityOptimizer-HRI2026/tree/integration/preview-planner-optimizer);
it has not been merged into `main`. Use the links rather than searching through
the older research files on the default branch.

**[Build and run instructions](https://github.com/ALMOWAFI/The-legibilityOptimizer-HRI2026/blob/integration/preview-planner-optimizer/docs/integration/connected-session.md#build-and-first-lab-checks)**

After building and sourcing the workspace, with the lab MoveIt stack running:

```bash
roslaunch hri_legibility_preview pipeline.launch \
  experiment_config:="$(rospack find hri_legible_motion)/config/experiment.yaml"
```

Check one trial in another sourced terminal:

```bash
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition baseline
rosrun hri_legibility_preview check_pipeline.py --target-id 0 --condition legible
```

Run the checks one at a time. The planner publishes `/legibility/baseline`; the
optimizer publishes `/legibility/trajectory_proposal`. Neither node executes
motion. [Exact topic/message contract](https://github.com/ALMOWAFI/The-legibilityOptimizer-HRI2026/blob/integration/preview-planner-optimizer/docs/integration/README.md).

**Current status:** 140 offline tests passed for the node cleanup; live ROS/robot
operation is unverified. The professor's fixed-height 2D requirement is the next
code change. The current code does not yet enforce a plane, and the plane height
has not been selected.

The [research manuscript and offline prototype guide](docs/research/README.md)
are background material. The numerical solver, configuration, messages and tests
are supporting files for the nodes. The game, interface and execution are
separate team components.
