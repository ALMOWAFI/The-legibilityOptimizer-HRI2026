# UR5e Legibility Optimizer

Standalone research workspace for the **UR5e FK-based joint-trajectory legibility
optimizer**. This repository holds the prototype, technical manuscript, offline
adapter, tests and validation notes while we decide how to integrate it into the
[group project](https://github.com/ALMOWAFI/HRI2026-Legibility-Of-Robots-Motion).

**Current status: offline prototype.** No robot commands, ROS launch configuration
or controller tools are included. Numerical optimization success is separate from
physical trajectory validity. The robot model is confirmed as UR5e; actual lab
kinematics, frames, limits, planning group and validation still need connecting.

The eventual architecture is:

```text
MoveIt / RRT-Connect baseline
    -> standalone legibility optimizer
    -> retiming
    -> robot-specific trajectory validation
    -> explicit execution
```

The optimizer uses joint positions as decision variables and injected FK to
evaluate visible end-effector motion. It preserves start/end configurations and
nominal timestamps, enforces joint/local-deviation/motion-cost constraints, and
returns a diagnostic candidate. It never executes a trajectory.

| Location | Contents |
|---|---|
| [`experimental/legibility_optimizer`](experimental/legibility_optimizer) | Pure numerical optimizer, offline ROS 1 message adapter, explicit missing robot interfaces and synthetic demo |
| [`experimental/requirements-legibility.txt`](experimental/requirements-legibility.txt) | NumPy/SciPy dependencies; tests use standard-library `unittest` |
| [`tests/experimental`](tests/experimental) | Numerical acceptance/rejection, trajectory/message handling and execution-isolation tests |
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
python -m pip install -r experimental/requirements-legibility.txt
python -m unittest discover -s tests/experimental -v
python -m experimental.legibility_optimizer.demo --output build/legibility-demo
```

Optional plots require Matplotlib. The demo uses synthetic identity FK, not UR5e
kinematics. See the [research README](docs/research/README.md) for plot commands.

## Build the manuscript

With `pdflatex` installed:

```sh
python docs/research/build_manuscript.py
```

The script verifies all seven source listings and builds
`build/research/legibility_optimizer_ur5e.pdf` in two passes. Generated artifacts
stay under the ignored `build/` directory.

## Future group integration

Development stays here for now. A first integration milestone is reading a saved
UR5e MoveIt baseline and comparing baseline/candidate FK paths offline. The group
project's motion and experiment nodes remain separate work; no code from this
repository is wired into their hardware execution path.

The original supplied report called the robot "UD5". The owner subsequently
confirmed **UR5e**, so current project/manuscript naming uses UR5e while provenance
retains the original report title.
