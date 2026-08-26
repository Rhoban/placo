<div align="center">

<img width="360" src="https://placo.readthedocs.io/en/latest/_static/placo.png" alt="PlaCo" />

<h3>Rhoban's Planning and Control library</h3>

<p>
  QP-based task-space inverse <b>kinematics</b> and <b>dynamics</b> for robots.<br />
  Written in C++ for runtime performance, with Python bindings for fast prototyping.
</p>

<p>
  <a href="https://placo.readthedocs.io/en/latest/"><img alt="Documentation" src="https://img.shields.io/readthedocs/placo?label=docs" /></a>
  <a href="https://pypi.org/project/placo/"><img alt="PyPI" src="https://img.shields.io/pypi/v/placo" /></a>
  <a href="https://pypi.org/project/placo/"><img alt="Python versions" src="https://img.shields.io/badge/python-3.10%2B-blue" /></a>
  <a href="https://github.com/Rhoban/placo/actions/workflows/wheels.yml"><img alt="Build" src="https://github.com/Rhoban/placo/actions/workflows/wheels.yml/badge.svg" /></a>
  <a href="https://arxiv.org/abs/2511.06141"><img alt="arXiv" src="https://img.shields.io/badge/arXiv-2511.06141-b31b1b" /></a>
  <a href="LICENSE"><img alt="License" src="https://img.shields.io/pypi/l/placo" /></a>
</p>

<p>
  <a href="https://placo.readthedocs.io/en/latest/"><b>Documentation</b></a> &nbsp;·&nbsp;
  <a href="https://placo.readthedocs.io/en/latest/kinematics/getting_started.html"><b>Getting started</b></a> &nbsp;·&nbsp;
  <a href="https://github.com/rhoban/placo-examples"><b>Examples repository</b></a> &nbsp;·&nbsp;
  <a href="https://placo.readthedocs.io/en/latest/basics/installation_source.html"><b>Build from source</b></a> &nbsp;·&nbsp;
  <a href="https://arxiv.org/abs/2511.06141"><b>Paper</b></a>
</p>

```sh
pip install placo
```

</div>

---

## What is PlaCo?

PlaCo is Rhoban's planning and control library. It is built on top of
[pinocchio](https://github.com/stack-of-tasks/pinocchio) and the
[eiquadprog](https://github.com/stack-of-tasks/eiquadprog) QP solver, and provides a high-level API to express
whole-body control problems as a set of **tasks** and **constraints**, which are assembled into a quadratic
program and solved for you.

<table>
<tr>
<td width="50%" valign="top" align="center">
<a href="https://github.com/Rhoban/placo-examples/blob/master/kinematics/videos/quadruped_targets.mp4?raw=true"><img src="https://github.com/Rhoban/placo-examples/blob/master/kinematics/videos/quadruped_targets.gif?raw=true" alt="Quadruped demo" /></a>
<h3>Task-Space Inverse Kinematics</h3>
<p>Specify tasks for constrained inverse kinematics (IK): frames, center of mass, joints, gears, wheels…</p>
<p>
  <a href="https://placo.readthedocs.io/en/latest/kinematics/getting_started.html">Documentation</a> &nbsp;·&nbsp;
  <a href="https://placo.readthedocs.io/en/latest/kinematics/examples_gallery.html">Examples gallery</a>
</p>
</td>
<td width="50%" valign="top" align="center">
<a href="https://github.com/Rhoban/placo-examples/blob/master/dynamics/videos/megabot.mp4?raw=true"><img src="https://github.com/Rhoban/placo-examples/blob/master/dynamics/videos/megabot.gif?raw=true" alt="Megabot demo" /></a>
<h3>Task-Space Inverse Dynamics</h3>
<p>Specify tasks for constrained inverse dynamics (ID), including contacts, torque and velocity limits.</p>
<p>
  <a href="https://placo.readthedocs.io/en/latest/dynamics/getting_started.html">Documentation</a> &nbsp;·&nbsp;
  <a href="https://placo.readthedocs.io/en/latest/dynamics/examples_gallery.html">Examples gallery</a>
</p>
</td>
</tr>
</table>

## Installation

PlaCo is distributed as a wheel for Linux and macOS (x86-64 and arm64), and requires Python 3.10 or later:

```sh
pip install placo
```

It can also be [built from sources](https://placo.readthedocs.io/en/latest/basics/installation_source.html),
which is required to use the C++ API.

## A first taste

```python
import numpy as np
import placo

# Load a robot and create the kinematics solver
robot = placo.RobotWrapper("models/6axis/")
solver = placo.KinematicsSolver(robot)
solver.mask_fbase(True)  # the robot is anchored to the ground

# Ask the "effector" frame to reach a given pose
effector_task = solver.add_frame_task("effector", np.eye(4))
effector_task.configure("effector", "soft", 1.0, 1.0)

# Solve the IK, and integrate the result in the robot state
robot.update_kinematics()
solver.solve(True)
```

Head to the [getting started guide](https://placo.readthedocs.io/en/latest/kinematics/getting_started.html)
for a walkthrough, or to the [examples repository](https://github.com/rhoban/placo-examples) for runnable
robots and scenarios.

## Resources

* [Documentation](https://placo.readthedocs.io/en/latest/) — guides and full API reference
* [Examples repository](https://github.com/rhoban/placo-examples) — runnable demos, models and videos
* [Paper](https://arxiv.org/abs/2511.06141) — *PlaCo: a QP-based robot planning and control framework*
* [Issues](https://github.com/Rhoban/placo/issues) — bug reports and feature requests

## Citation

If you use PlaCo in your research, please cite the following paper:

```bibtex
@misc{duclusaud2025placo,
      title={PlaCo: a QP-based robot planning and control framework},
      author={Marc Duclusaud and Grégoire Passault and Vincent Padois and Olivier Ly},
      year={2025},
      eprint={2511.06141},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2511.06141},
}
```

## License

PlaCo is released under the [MIT License](LICENSE).
