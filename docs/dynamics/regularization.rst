Regularization
==============

In the case of underconstrained tasks, there might be many solutions to the problem: a choice has to be
made about the preferred solution.
Regularization is the addition of a very low cost to the objective function, in order to favour some solutions
over others.
Do not hesitate to have a look at :ref:`kinematics regularization <regularization>`.

Torque regularization
---------------------

In the dynamics solver, the torque can be expressed as the other decision variables.
The (squared) torque is regularized by default, meaning that :math:`\sum \tau_i^2` will be minimized,
with a default weight of :math:`10^{-3}`.
This weight can be adjusted with :func:`solver.torque_cost <placo.DynamicsSolver.torque_cost>`:

.. code-block:: python

    # Weight for the torque regularization (1e-3 by default)
    solver.torque_cost = 1e-2

Pose regularization
-------------------

Another type of regularization is to use a :doc:`joint task <joints_task>` or a
:doc:`position, orientation or frame task <position_orientation_frame_task>` to regularize
the pose of the robot.

To do that, simply give a very low weight to the task, for example:

.. code-block:: python

    # Adding a task with a very low priority to take the DoFs back to 0
    joints_task = solver.add_joints_task()
    joints_task.set_joints({
        joint: 0.0
        for joint in robot.joint_names()
    })
    joints_task.configure("joints_regularization", "soft", 1e-5)

