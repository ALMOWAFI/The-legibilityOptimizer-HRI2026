# UR5e Legibility Optimizer

ROS 1 planner and **FK-based joint-trajectory legibility optimizer** for the
UR5e group project. Development stays in this separate repository; the group's
working placement and hardware tools have not been modified.

**Status: topic pipeline implemented and tested offline.** The numerical solver
is real; ROS transport, MoveIt planning and FK are substituted in integration
tests. Catkin compilation and live UR5e operation remain unverified. No controller
client or execution node is included.

For lab setup, start with the [connected-session guide](docs/integration/connected-session.md).
Use branch `integration/preview-planner-optimizer` and the companion group branch
`integration/rrt-optimizer-session`; these changes are in draft PRs, not merged into main.

The implemented planning handoff is:

```text
/hri/block_layout + /legibility/trial_request
    -> planner_node: MoveIt / RRT-Connect
    -> /legibility/baseline
    -> optimizer_node: baseline bypass OR accepted FK optimization
    -> /legibility/trajectory_proposal
    -> external retiming, physical validation and explicit execution (still required)
```

The optimizer uses joint positions as decision variables and injected FK to
evaluate visible end-effector motion. It preserves start/end configurations and
nominal timestamps, enforces joint/local-deviation/motion-cost constraints, and
returns a diagnostic candidate. It never executes a trajectory.

Both accepted conditions publish `TrajectoryProposal`. Baseline trials bypass
optimization; legible trials require numerical acceptance and also publish detailed
metrics on `/legibility/optimized_candidate`. Every proposal requires retiming and
physical validation and has `execution_ready=false`. The publisher reads the
same block YAML as the placement tool. RRT-Connect does not guarantee a straight
Cartesian reach; the experiment's `predictable` label maps to this baseline.

## Lab entry point

After building and sourcing both integration branches with the actual lab
ROS 1/MoveIt environment, use one shared, current experiment configuration:

```bash
export HRI_CONFIG="$(rospack find hri_legible_motion)/config/experiment.yaml"
roslaunch hri_legibility_preview group_pipeline.launch \
  experiment_config:="$HRI_CONFIG" display:=false
```

This launches the layout publisher, planner and optimizer. It sends no robot
motion commands. The [lab guide](docs/integration/connected-session.md#build-and-first-lab-checks)
contains the build steps, trial requests and expected output.

| Launch file | Purpose |
|---|---|
| `group_pipeline.launch` | Lab integration entry point; shared group YAML and optional participant display. |
| `pipeline.launch` | Standalone topic pipeline, without the group display. Supply the active config explicitly; its default is an archived lab snapshot. |
| `preview.launch` | Developer service/preview entry point; requires explicit config and a separate layout/trial source for topic use. |

Launch only one of these at a time: they use the same planner/optimizer node names.
The group's legacy `hri.launch`/`motion_node.py` and optional `/trial_start` bridge
are not part of the connected lab flow. The new session uses
`experiment_node.py --pipeline`; it needs the separately implemented validator
and executor described in the lab guide.

Research manuscripts, historical reviews, synthetic demos and tests are retained
for provenance and verification. They are not loaded by the lab launch.

| Location | Contents |
|---|---|
| [`experimental/legibility_optimizer`](experimental/legibility_optimizer) | Pure numerical optimizer, offline ROS 1 message adapter, explicit missing robot interfaces and synthetic demo |
| [`experimental/requirements-legibility.txt`](experimental/requirements-legibility.txt) | NumPy/SciPy dependencies; tests use standard-library `unittest` |
| [`tests/experimental`](tests/experimental) | Numerical acceptance/rejection, trajectory/message handling and execution-isolation tests |
| [`legibility_ros`](legibility_ros) | Model FK, planner/optimizer services, topic subscribers/publishers, optional sampled preview checks |
| [`scripts`](scripts) | Planner, optimizer, persistent layout publisher and manual trial sender |
| [`msg`](msg), [`srv`](srv), [`launch`](launch) | Typed trial/layout/trajectory handoffs and launch wiring |
| [`tests/ros1`](tests/ros1) | ROS protocol, planner request, topic handoff and real numerical optimization tests |
| [`config/experiment.lab.yaml`](config/experiment.lab.yaml) | User-supplied lab configuration snapshot; replace with the active group file when launching |
| [`docs/research/legibility_optimizer_ur5e.tex`](docs/research/legibility_optimizer_ur5e.tex) | Repaired manuscript with the actual implementation and tests included |
| [`docs/research/README.md`](docs/research/README.md) | Detailed API contract, setup, build commands and verification results |
| [`docs/research/integration.md`](docs/research/integration.md) | Historical group-repository audit and proposed future integration boundary |
| [`docs/research/provenance.md`](docs/research/provenance.md) | Supplied-report provenance, citations, adaptations and reuse record |

## Run offline

Use an isolated Python environment from this repository's root:

```sh
python -m venv .venv-legibility
# Linux/macOS: source .venv-legibility/bin/activate
# PowerShell: .venv-legibility\Scripts\Activate.ps1
python -m pip install -r requirements-dev.txt
python -m unittest discover -s tests/experimental -v
python -m unittest discover -s tests/ros1 -v
python -m experimental.legibility_optimizer.demo --output build/legibility-demo
```

Optional plots require Matplotlib. The demo uses synthetic identity FK, not UR5e
kinematics. See the [research README](docs/research/README.md) for plot commands.

## Build the manuscript

With `pdflatex` installed:

```sh
python docs/research/build_manuscript.py
```

The script verifies all source listings and builds
`build/research/legibility_optimizer_ur5e.pdf` in two passes. Generated artifacts
stay under the ignored `build/` directory.

## Group integration

The [connected-session guide](docs/integration/connected-session.md) is the current
lab workflow. The [protocol reference](docs/integration/README.md) covers topic and
service details. The [older branch review](docs/integration/group-branch-review.md)
records historical findings, not current setup instructions. The
[change summary](docs/integration/changes.md) lists files for review; the
[verification record](docs/integration/verification.md) distinguishes offline checks
from outstanding lab validation. No controller implementation is included.

The original supplied report called the robot "UD5". The owner subsequently
confirmed **UR5e**, so current project/manuscript naming uses UR5e while provenance
retains the original report title.
