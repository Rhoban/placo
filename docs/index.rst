
Welcome to PlaCo's documentation
================================

.. admonition:: Whas it it?
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/6axis_trajectory.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/6axis_trajectory.py`

    PlaCo is an implementation for whole-body inverse kinematics and dynamics based on
    Quadratic Programming solver.

    Written in C++, it exposes Python bindings for easy prototyping, and can still be used
    as a standalone library for embedded applications.

Outline
-------

.. toctree::
  :maxdepth: 3

  basics/index
  kinematics/index
  dynamics/index
  placo/summary

Acknowledgements
----------------

PlaCo's rigid body model (kinematics, dynamics, URDF and MJCF loading) is a lighter, standalone implementation
written with `pinocchio <https://github.com/stack-of-tasks/pinocchio>`_ as its reference, on top of which PlaCo
was first built: it follows pinocchio's conventions and formulas, and was validated against it. Collisions and
distances are computed with `coal <https://github.com/coal-library/coal>`_.

Links
-----

* `GitHub repository <https://github.com/rhoban/placo>`_
