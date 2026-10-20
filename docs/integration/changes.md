# File-by-file review summary

| File | Change and purpose |
|---|---|
| `experimental/legibility_optimizer/sampled_optimizer.py` | Support-point solve, offset interpolation onto the unchanged full baseline grid, independent full-grid acceptance checks. Core report equations remain unchanged. |
| `legibility_ros/__init__.py` | Separate ROS integration package; numerical code stays free of ROS imports. |
| `legibility_ros/contracts.py` | Full-state FK requests, exact bounded cache, frame/link/error checks, loaded limits, derivative clearing, checked retiming and optional discrete validation. |
| `legibility_ros/layout.py` | Typed layout conversion, canonical content digest, frame/ID/pose/workspace checks and frozen per-trial snapshot. |
| `legibility_ros/runtime.py` | Actual MoveIt RRT-Connect/Cartesian planning and FK-based optimization operations; explicit namespace, runtime planner verification and separate numerical/preview modes. |
| `legibility_ros/pipeline.py` | Optional retime/sample-check preview orchestration; rejection retains numerical status and never authorizes execution. |
| `legibility_ros/topics.py` | Layout/trial subscribers, baseline/candidate publishers, group TrialInfo bridge, busy/duplicate handling and explicit rejection status. |
| `legibility_ros/experiment_client.py` | Optional group-session client: unique attempts, config/layout binding, bounded preparation, separate validation and operator-authorized downstream requests, correlated measured feedback. No controller client. |
| `scripts/planner_node.py` | Planner node entry point. |
| `scripts/optimizer_node.py` | Optimizer node entry point. |
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
| `srv/ValidateTrajectory.srv`, `srv/RequestMotion.srv` | Contracts for the teammate's separate validator/executor; providers are intentionally not guessed or implemented. |
| `launch/group_pipeline.launch` | Connects the standalone pipeline with the group's existing display; excludes the placeholder motion node and legacy trial bridge. |
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
| `README.md`, `docs/integration/*` | Current status, connection contract, launch/build/test commands, branch review and downstream ownership. |
| `docs/research/*` | Historical audit marked as such; manuscript integration status updated while preserving original mathematics, citations and listing files. |

No group hardware code was changed. No controller/execution implementation or
generic UR5 substitute model was added. No robot or participant trial was run.

The companion group branch changes `scripts/experiment_node.py` to add optional
`--pipeline` and `--config`, skips cached trajectories in that mode, reuses
scheduling/display/scoring, logs pipeline attempts, and uses measured motion
events. It adds `test/test_pipeline_session.py` and `docs/connected_pipeline.md`,
plus a README link. The original dirty group checkout was not modified.
