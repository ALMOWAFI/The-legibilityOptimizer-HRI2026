# UR5e Legibility Optimizer

ROS 1 planner and **FK-based joint-trajectory legibility optimizer** for the
UR5e group project. Development stays in this separate repository; the group's
working placement and hardware tools have not been modified.

**Status: topic pipeline implemented and tested offline.** The numerical solver
is real; ROS transport, MoveIt planning and FK are substituted in integration
tests. Catkin compilation and live UR5e operation remain unverified. No controller
client or execution node is included.

The implemented handoff is:

```text
/hri/block_layout + /legibility/trial_request (or optional /trial_start)
    -> planner_node: MoveIt / RRT-Connect
    -> /legibility/baseline
    -> optimizer_node: FK-based numerical optimization
    -> /legibility/optimized_candidate
    -> teammate's retiming, physical validation and explicit execution stage
```

The optimizer uses joint positions as decision variables and injected FK to
evaluate visible end-effector motion. It preserves start/end configurations and
nominal timestamps, enforces joint/local-deviation/motion-cost constraints, and
returns a diagnostic candidate. It never executes a trajectory.

Predictable (`baseline`) trials publish a baseline and bypass optimization.
Legible trials publish an optimized candidate only after numerical acceptance.
Every candidate explicitly requires retiming and physical validation and has
`execution_ready=false`. See the [pipeline guide](docs/integration/README.md) for
topic types, launch commands, the optional group-message bridge and the downstream
contract. The publisher uses the same block YAML as the placement tool.

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

The [integration guide](docs/integration/README.md) describes the implemented
topic pipeline and what the downstream teammate must consume. The
[branch review](docs/integration/group-branch-review.md) distinguishes published
group functionality from planned interfaces. The [change summary](docs/integration/changes.md)
lists files for review. No code here is connected to the group's controller path.

The original supplied report called the robot "UD5". The owner subsequently
confirmed **UR5e**, so current project/manuscript naming uses UR5e while provenance
retains the original report title.
