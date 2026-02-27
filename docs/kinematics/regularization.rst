Regularization
==============

.. _regularization:

Default L2 regularization
-------------------------

Specified tasks might lead to an under-constrained problem, where an infinite number of solutions exist.
In that case, the solver will try to minimize the norm of the configuration variation, which will lead to
a solution that is as close as possible to the current configuration.

This is because a very low weighted cost is always present in front of the norm of the configuration variation
:math:`\lVert \Delta q \rVert^2`.

.. admonition:: 6-axis regularization (default)

    In this example, the robot is following a target 3D position. The problem is under-constrained, so the default
    regularization will apply.

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/6axis_regularization_default.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/6axis_regularization.py`

Custom L2 regularization
------------------------

You can add your own regularization by adding a :func:`RegularizationTask <placo.RegularizationTask>`:

.. code-block:: python

    # Adding a custom regularization task
    regularization_task = solver.add_regularization_task(1e-4)

The greater is the provided regularization value, the more expansive it will be for the solver to move
any joint.

The task weight can be updated with:

.. code-block:: python

    # Updating regularization weight
    regularization_task.set_weight(1e-6)

Also, specific values can be set for given joints:

.. code-block:: python

    # Setting a high regularization for r1 only
    regularization_task.set_joint_weight("r1", 1e2)

.. admonition:: 6-axis regularization (custom strong L2)

    If you pass ``--strong_l2`` to the following example, a strong :math:`10^{2}` L2 regularization will be used.
    This will create a strong cost against the motion and cause the robot to lag behind its target.

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/6axis_regularization_strong_l2.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/6axis_regularization.py`

Posture regularization
----------------------

Another way to regularize the problem is using a posture regularization. This can be achieved with *e.g*
a :func:`JointsTask <placo.JointsTask>`:

.. code-block:: python

    # Adding a posture regularization task
    joints_task = solver.add_joints_task()
    joints_task.set_joints({
        joint: 0.0
        for joint in robot.joint_names()
    })
    joints_task.configure("posture", "soft", 1e-6)


Here, the joints task is setting a target to zero for all joints with a small soft weight of *1e-6*.
When no tasks are specified, the joints will go back to those positions.

.. admonition:: 6-axis regularization (posture)

    If you pass ``--posture`` to the following example, a joint task with :math:`10^{-6}` weight will be used.
    Note that the pose of the robot is slightly different from the default regularization

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/6axis_regularization_posture.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/6axis_regularization.py`

Kinetic energy regularization
-----------------------------

Another possible regularization is to minimize the kinetic energy in the system. This can be done by using
a :func:`KineticEnergyRegularizationTask <placo.KineticEnergyRegularizationTask>`:

.. code-block:: python

    # Adding a kinetic energy regularization task
    kinetic_energy_task = solver.add_kinetic_energy_regularization_task(1e-6)

This will minimize :math:`\frac{1}{2} \dot{q}^T M \dot{q}`, where :math:`M` is the inertia matrix of the robot.

.. note::

    When using this regularization, and to ensure that the task has the unit of a kinetic energy,
    you have to specify the :math:`\Delta t` between two successive calls to the solver.
    This can be done by setting ``solver.dt``:

    .. code-block:: python

        # Setting solver.dt is required to use kinetic energy regularization
        solver.dt = 0.01

.. admonition:: 6-axis regularization (kinetic energy)

  If you pass ``--kinetic`` to the following example, a kinetic energy regularization task with :math:`10^{-6}`
  weight will be used. Since the energy is weighted by the mass matrix, the end of the arm will
  be more used than the base.

  .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/6axis_regularization_kinetic.mp4
      :autoplay:
      :muted:
      :loop:

  :example:`kinematics/6axis_regularization.py`


Manipulability regularization
-----------------------------

Manipulability is a well-known metric establishing the ability of the robot to produce motion in all the
directions (for position and/or orientation) in the task-space.
Formally, it is the volume of the ellipsoid obtained by mapping unit-length motion in joint-space to task-space
(through the relevant Jacobian).

You can use a :func:`ManipulabilityTask <placo.ManipulabilityTask>`:

.. code-block:: python

    manipulability = solver.add_manipulability_task("effector", "both", 1.0)
    manipulability.configure("manipulability", "soft", 1e-3)

Here:

* ``"effector"`` is the frame for which manipulability should be maximized (in ``LOCAL`` reference frame)
* ``"both"`` means that the manipulability for both position and orientation will be used. The values ``"position"``
  or ``"orientation"`` can be used to only compute the manipulability for respectively position and orientation
* The last parameter, ``1.0`` is a value that will be added on the Hessian of the objective function. Since
  maximizing the manipulability is a first-order task, it only provides a gradient direction and needs to be regularized.

.. admonition:: Math details

    To compute the manipulability derivatives, kinematics hessians need to be computed.
    See `https://arxiv.org/abs/2002.11901 <https://arxiv.org/abs/2002.11901>`_.

.. admonition:: manipulability

  If you pass ``--manipulability`` to the 6axis regularization example, the manipulability will be maximized
  during the process.

  :example:`kinematics/6axis_regularization.py`
