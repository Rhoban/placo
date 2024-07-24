# PlaCo

PlaCo is Rhoban's planning and control library.
Its main features are:

* Task-space Inverse Kinematics with constraints,
* Task-space Inverse Dynamics with constraints,
* QP problem formulation,
* Built on the top of [pinocchio](https://github.com/stack-of-tasks/pinocchio)
* Written in C++ with Python bindings

[![Megabot demo](https://github.com/Rhoban/placo-examples/blob/master/kinematics/videos/quadruped_targets.gif?raw=true)](https://github.com/Rhoban/placo-examples/blob/master/kinematics/videos/quadruped_targets.mp4?raw=true)

*Inverse Kinematics Example: a quadruped robot hitting targets with a leg while keeping its three legs on the ground*

[source code (quadruped_targets.py)](https://github.com/Rhoban/placo-examples/blob/master/kinematics/quadruped_targets.py) / [more kinematics examples](https://placo.readthedocs.io/en/latest/kinematics/examples_gallery.html)

[![Megabot demo](https://github.com/Rhoban/placo-examples/blob/master/dynamics/videos/megabot.gif?raw=true)](https://github.com/Rhoban/placo-examples/blob/master/dynamics/videos/megabot.mp4?raw=true)

*Inverse Dynamics Example: a quadruped with many loop closure joints*

[source code (megabot.py)](https://github.com/Rhoban/placo-examples/blob/master/dynamics/megabot.py) / [more dynamics examples](https://placo.readthedocs.io/en/latest/dynamics/examples_gallery.html)

## Installing

PlaCo is available from [pip](https://placo.readthedocs.io/en/latest/basics/installation_pip.html),
or can be [built from sources](https://placo.readthedocs.io/en/latest/basics/installation_source.html).

## Documentation

Here is the [official documentation](https://placo.readthedocs.io/en/latest/)

You can also find many examples in the [placo-examples](https://github.com/rhoban/placo-examples)
repository.