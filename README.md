<img width="400" src="https://placo.readthedocs.io/en/latest/_static/placo.png" />

## Planning & Control

PlaCo is Rhoban's planning and control library. It is built on the top of [pinocchio](https://github.com/stack-of-tasks/pinocchio), [eiquadprog](https://github.com/stack-of-tasks/eiquadprog) QP solver, and fully written in C++ with Python bindings, allowing fast prototyping with good runtime performances. It features task-space inverse kinematics and dynamics (see below) high-level API for whole-body control tasks.

### Task-Space Inverse Kinematics

[![Quadruoped demo](https://github.com/Rhoban/placo-examples/blob/master/kinematics/videos/quadruped_targets.gif?raw=true)](https://github.com/Rhoban/placo-examples/blob/master/kinematics/videos/quadruped_targets.mp4?raw=true)

High-level API to specify tasks for constrained inverse kinematics (IK).

- [See documentation](https://placo.readthedocs.io/en/latest/kinematics/getting_started.html)
- [Examples](https://placo.readthedocs.io/en/latest/kinematics/examples_gallery.html)

### Task-Space Inverse Dynamics

[![Megabot demo](https://github.com/Rhoban/placo-examples/blob/master/dynamics/videos/megabot.gif?raw=true)](https://github.com/Rhoban/placo-examples/blob/master/dynamics/videos/megabot.mp4?raw=true)

High-level API to specify tasks for constrained inverse dynamics (ID).

- [See documentation](https://placo.readthedocs.io/en/latest/dynamics/getting_started.html)
- [Examples](https://placo.readthedocs.io/en/latest/dynamics/examples_gallery.html)


## Installing

PlaCo can be installed from ``pip``

```
pip install placo
```

Or [built from sources](https://placo.readthedocs.io/en/latest/basics/installation_source.html)

## Resources

* [Documentation](https://placo.readthedocs.io/en/latest/)
* [Examples](https://github.com/rhoban/placo-examples) repository
