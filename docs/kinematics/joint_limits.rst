Joint and velocity limits
=========================

Joint limits
------------

Joint limits are **enabled by default** in the kinematics solver. They can be enabled/disabled
by using the :func:`enable_joint_limits <placo.KinematicsSolver.enable_joint_limits>`:

.. code-block:: python

    # Enables / disables the joint limits (enabled by default)
    solver.enable_joint_limits(True)

Joint limits are loaded by default from the URDF file and can be overriden on the
:ref:`robot wrapper <joint_limits>`.

Velocity limits
---------------

Velocity limits are **disabled by default**. They can be enabled/disabled by using the
:func:`enable_velocity_limits <placo.KinematicsSolver.enable_velocity_limits>`:

.. code-block:: python

    # Enables / disables the velocity limits (disabled by default)
    solver.enable_velocity_limits(True)

Velocity limits are loaded by default from the URDF file and can be overriden on the
:ref:`robot wrapper <joint_limits>`.

.. note::

    When using velocity limits, you have to specify the :math:`\Delta t` between two
    successive calls to the solver. This can be done by setting ``solver.dt``:

    .. code-block:: python

        # Setting solver.dt is required to use velocity limits
        solver.dt = 0.01 

Retrieving velocity
-------------------

The kinematics solver actually produces a :math:`\Delta q` vector that is a small
variation of the current joint configuration.
This vector is returned by the :func:`solve <placo.KinematicsSolver.solve>` method, and can be used to compute
a velocity:

.. code-block:: python

    # Solves the kinematics problem
    dq = solver.solve()

    # Computes the velocity
    v = dq / dt

.. note::

    When the solved value is applied (by passing `True` in `solver.solve(True)`), the robot state
    (`robot.state.qd`) is updated with the computed velocity.