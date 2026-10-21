# File-by-file review summary

| File | Change and purpose |
|---|---|
| `experimental/legibility_optimizer/sampled_optimizer.py` | Support-point solve, offset interpolation onto the unchanged full baseline grid, independent full-grid acceptance checks. Core report equations remain unchanged. |
| `legibility_ros/__init__.py` | Separate ROS integration package; numerical code stays free of ROS imports. |
| `legibility_ros/contracts.py` | Full-state FK requests, exact bounded cache, frame/link/error checks, loaded limits, derivative clearing, checked retiming and optional discrete validation. |
| `legibility_ros/layout.py` | Typed layout conversion, canonical content digest, frame/ID/pose/workspace checks and frozen per-trial snapshot. |
| `legibility_ros/runtime.py` | Shared loaded robot configuration, FK and validation context. Planning and optimizer callbacks live in the two node scripts. |
| `legibility_ros/pipeline.py` | Optional retime/sample-check preview orchestration; rejection retains numerical status and never authorizes execution. |
| `legibility_ros/topics.py` | Shared status reporting and busy/duplicate-trial handling. Node-specific subscriptions and publications live in their node scripts. |
| `legibility_ros/experiment_client.py` | Optional group-session client: unique attempts, config/layout binding, bounded preparation, separate validation and operator-authorized downstream requests, correlated measured feedback. No controller client. |
| `scripts/planner_node.py` | Full planner node: layout/trial subscriptions, loaded RRT-Connect selection, MoveIt planning, baseline publication and RViz preview. |
| `scripts/optimizer_node.py` | Full optimizer node: baseline subscription, FK-based numerical optimization, acceptance/rejection and proposal publication. Calls the standalone mathematical solver. |
| `scripts/layout_publisher.py` | Persistent latched layout publisher reading the same YAML as group placement. |
| `scripts/send_trial.py` | Manual planning/optimization trigger with layout binding; no execution option. |
| `scripts/check_pipeline.py`, `legibility_ros/flow_check.py` | Operator check of actual node/topic registrations, message types, transport, latched layout and correlated per-trial results. Sends one planning-only trial; rejects missing, failed or mismatched output. |
| `msg/Block.msg` | Paired block identity and cube-center pose. |
| `msg/BlockLayout.msg` | Layout frame, content ID, blocks and tool goal height. |
| `msg/TrialRequest.msg` | Trial, intended target, condition and optional expected layout ID. |
| `msg/BaselinePlan.msg` | Atomic baseline/state/trial/layout/model handoff. |
| `msg/OptimizedCandidate.msg` | Candidate plus source, numerical metrics and mandatory downstream-validation flags. |
| `msg/TrajectoryProposal.msg` | Common downstream handoff for both baseline and optimized conditions, always requiring physical validation. |
| `msg/MotionFeedback.msg` | Command/trial/token-correlated measured motion events from an external provider. |
| `srv/PlanPreview.srv` | Optional direct planning/preview service. |
| `srv/OptimizeLegibility.srv` | Optional direct numerical/preview optimization service. |
| `srv/ValidateTrajectory.srv`, `srv/RequestMotion.srv` | Separate downstream services; requests include human confirmation and a bounded dispatch deadline. Rebuild both packages after this schema change. |
| `launch/group_pipeline.launch` | Starts the standalone pipeline, downstream node with execution disabled, and optional group display; excludes the placeholder motion node and legacy trial bridge. |
| `launch/pipeline.launch` | Persistent layout plus both nodes; RRT-Connect default and optional group subscription. |
| `launch/preview.launch` | Two-node service-oriented preview launch; explicit config path. |
| `config/experiment.lab.yaml` | User-supplied configuration snapshot, including lab frames and IDs; Git may normalize text line endings. |
| `CMakeLists.txt`, `package.xml`, `setup.py` | Catkin message/service generation, Python package installation and declared dependencies. |
| `requirements-dev.txt` | Offline numerical and ROS protocol test dependencies. |
| `tests/experimental/test_sampled_optimizer.py` | Full-grid endpoint, timing, budget, bounds and rejection tests. |
| `tests/ros1/test_preview.py` | FK, limits, retiming, discrete validation and execution-isolation tests. |
| `tests/ros1/test_runtime.py` | Actual node callback tests using substituted ROS boundaries. |
| `tests/ros1/test_topics.py` | End-to-end topic handoff with real SLSQP and simulated MoveIt/FK; failures, snapshots and duplicate trials. |
| `tests/ros1/test_flow_check.py` | Real-solver flow check, baseline bypass, missing data/registrations, wrong types, rejection, stale trials and corrupted output. |
| `tests/ros1/test_experiment_client.py` | Session-to-proposal real numerical solve, condition mapping, rejection, service absence, explicit authorization, single-use validation and feedback correlation. |
| `docs/integration/connected-session.md` | Two-repository build/run commands, exact downstream contract, timing/logging semantics and remaining lab work. |
| `legibility_execution/__init__.py` | Separate package boundary; never imported by planner/optimizer nodes. |
| `legibility_execution/trajectory.py` | Checked IPTP retiming/duration scaling, quintic joint extrema, sampled FK/collision/TCP checks, exact slowed reversal. |
| `legibility_execution/gate.py` | Single-use expiring validation IDs, deduplicated commands, explicit enable/confirmation gates, no queue or automatic retry, fault latch. |
| `legibility_execution/state.py` | Fresh arm-only joint measurements, timestamps and finite velocity checks; gripper motion cannot trigger onset. |
| `legibility_execution/ros_node.py` | Downstream ROS services, exact proposal subscription, model/scene/controller checks, confirmed action dispatch, measured completion, cancellation request on faults, checked return. |
| `scripts/executor_node.py` | Separate downstream entry point; execution disabled unless explicitly enabled. |
| `scripts/inspect_execution.py` | Read-only model fingerprint, collision-object and controller inspection. |
| `msg/ValidatedTrajectory.msg` | Exact retimed trajectory, validation/context IDs and report for preview/recording; never a motion trigger. |
| `launch/execution.launch`, `config/execution.yaml` | Disabled-by-default execution wiring and explicit missing lab commissioning fields. |
| `tests/execution/test_execution.py` | Polynomial overshoot, sampled validity, scene geometry, model/controller identity, tokens, measurements, deadlines, faults and return checks using fake robot boundaries. |
| `tests/ros1/test_downstream_preparation.py` | Real solver through downstream retiming/checks, full-state preservation, altered proposal and changed-scene rejection. |
| `docs/integration/execution.md` | Read-only checks, commissioning and run commands, supported controller behavior and validation limits. |
| `README.md`, `docs/integration/*` | Current status, connection contract, launch/build/test commands, branch review and downstream responsibilities. |
| `docs/research/*` | Historical audit marked as such; manuscript integration status updated while preserving original mathematics, citations and listing files. |

No existing group hardware code was changed. The new controller implementation
is isolated in the opt-in downstream package; no generic UR5 model was added.
No robot or participant trial was run. Calibrated model, measured scene and
controller commissioning still require the lab.

The companion group branch changes `scripts/experiment_node.py` to add optional
`--pipeline` and `--config`, skips cached trajectories in that mode, reuses
scheduling/display/scoring, logs pipeline attempts, and uses measured motion
events. It adds `test/test_pipeline_session.py` and `docs/connected_pipeline.md`,
plus a README link. The original dirty group checkout was not modified.
