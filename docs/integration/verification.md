# Recorded verification — 2026-10-21

| Check | Actual result |
|---|---|
| `python -m unittest discover -s tests/experimental -v` | 40 tests passed in 5.151 seconds. Real SLSQP solves, goal posteriors, exact endpoints, bounds/budget/deviation, timestamps and rejection cases. |
| `python -m unittest discover -s tests/ros1 -v` | 46 tests passed in 5.582 seconds. Real numeric optimization through topic/session callbacks, with simulated ROS transport, MoveIt responses and FK. Includes validation absence, explicit authorization and correlated feedback. |
| Companion group: `PYTHONPATH=src python -m unittest discover -s test -v` | 30 tests passed in 0.476 seconds, including six optional-session integration tests. |
| `python -m compileall -q experimental legibility_ros scripts tests` | Passed. |
| Python 3.8 grammar parsing | Latest check: 11 production Python files in `legibility_ros`/`scripts`, plus the companion group experiment script, parsed successfully. This is not a Python 3.8 runtime test. |
| Launch/package XML | Parsed successfully. |
| `python docs/research/build_manuscript.py` (prior manuscript check, 2026-10-20) | Two successful pdflatex passes; 27 pages, eight resolved listing companions. No reported unresolved references, overfull/underfull boxes or missing-character warnings. The session integration did not change those listing sources. |
| PDF visual inspection | New integration section, interpolation equations and start of the new listing inspected on page 25. |

Tests ran on Windows with CPython 3.11.9. Test fixtures use synthetic identity FK,
not UR5 or UR5e kinematics. The RRT tests verify configuration selection, MoveIt
request handling, failed-plan rejection and cleanup; they do not run OMPL itself.
The topic bus is an in-memory substitute and does not test TCPROS serialization.

ROS 1 `rospy`, catkin and generated ROS message modules are unavailable in this
Windows environment. The local WSL environment has no `catkin_make` command.
**Catkin compilation, real ROS subscriptions/publications, calibrated FK, live
MoveIt RRT-Connect, simulation and hardware execution have not been verified.**
No real model/scene/trajectory capture was successfully supplied from the lab.

The ordinary numerical module remains isolated from ROS and network operations.
Tests also check that the integration imports no controller/action client and
publishes only declared layout/trial/baseline/candidate/proposal/status/display messages.
Accepted candidates retain `execution_ready=false`. Final retiming, physical
validation and explicit execution belong to the downstream teammate. The optional
experiment client calls that teammate's services only through the separate
validation and operator-authorized motion steps; it contains no controller client.
