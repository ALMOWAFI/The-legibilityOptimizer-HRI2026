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
| `scripts/planner_node.py` | Planner node entry point. |
| `scripts/optimizer_node.py` | Optimizer node entry point. |
| `scripts/layout_publisher.py` | Persistent latched layout publisher reading the same YAML as group placement. |
| `scripts/send_trial.py` | Manual planning/optimization trigger with layout binding; no execution option. |
| `msg/Block.msg` | Paired block identity and cube-center pose. |
| `msg/BlockLayout.msg` | Layout frame, content ID, blocks and tool goal height. |
| `msg/TrialRequest.msg` | Trial, intended target, condition and optional expected layout ID. |
| `msg/BaselinePlan.msg` | Atomic baseline/state/trial/layout/model handoff. |
| `msg/OptimizedCandidate.msg` | Candidate plus source, numerical metrics and mandatory downstream-validation flags. |
| `srv/PlanPreview.srv` | Optional direct planning/preview service. |
| `srv/OptimizeLegibility.srv` | Optional direct numerical/preview optimization service. |
| `launch/pipeline.launch` | Persistent layout plus both nodes; RRT-Connect default and optional group subscription. |
| `launch/preview.launch` | Two-node service-oriented preview launch; explicit config path. |
| `config/experiment.lab.yaml` | User-supplied configuration snapshot, including lab frames and IDs; Git may normalize text line endings. |
| `CMakeLists.txt`, `package.xml`, `setup.py` | Catkin message/service generation, Python package installation and declared dependencies. |
| `requirements-dev.txt` | Offline numerical and ROS protocol test dependencies. |
| `tests/experimental/test_sampled_optimizer.py` | Full-grid endpoint, timing, budget, bounds and rejection tests. |
| `tests/ros1/test_preview.py` | FK, limits, retiming, discrete validation and execution-isolation tests. |
| `tests/ros1/test_runtime.py` | Actual node callback tests using substituted ROS boundaries. |
| `tests/ros1/test_topics.py` | End-to-end topic handoff with real SLSQP and simulated MoveIt/FK; failures, snapshots and duplicate trials. |
| `README.md`, `docs/integration/*` | Current status, connection contract, launch/build/test commands, branch review and downstream ownership. |
| `docs/research/*` | Historical audit marked as such; manuscript integration status updated while preserving original mathematics, citations and listing files. |

No group hardware code was changed. No controller/execution implementation or
generic UR5 substitute model was added. No robot or participant trial was run.
