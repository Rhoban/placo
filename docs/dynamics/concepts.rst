Tasks and constraints
=====================

Tasks
-----

As you saw in the :doc:`introduction example <getting_started>`, 
a task defines something that the robot should do, like "reaching a given target with the effector".
In the context of the dynamics solver, a task eventually boils down into expressing a desired
acceleration.

As explained in the :doc:`introduction <dynamics/introduction>`, a task structure will look like:

:math:`\ddot x^{desired} = K_p (x^{task} - x) + K_d (\dot x^{task} - \dot x) + \ddot x^{task}`.

Where the :math:`task` superscript denotes the values produced by the task, and :math:`x` is the quantity
you want to control (joint position, effector position, effector orientation etc.).
In a :doc:`joint task <dynamics/joints_task>`, these values are simply the desired joint position, velocity and acceleration,
*i.e* :math:`x = q`.

Each task has a :func:`kp <placo.DynamicsTask.kp>` attribute, allowing to adjust the proportional gain and adjust
the task's "stiffness".
By default, the derivative gain :func:`kd <placo.DynamicsTask.kd>` is set to :math:`2 \sqrt{K_p}` to have a
`critically damped system <https://en.wikipedia.org/wiki/Damping>`_.
To change this behaviour, you can set the :func:`kd <placo.DynamicsTask.kd>` attribute to the desired value
(setting it to a negative value will restore the default critical damping behaviour).

+------------------+------------------------------------------------------------------------+
| Priority         | Description                                                            | 
+==================+========================================================================+
| ``hard``         | Task that must be satisfied by the solver (error should be brought     |
|                  | to zero).                                                              |
|                  | If it is not possible, the solver will fail.                           |
+------------------+------------------------------------------------------------------------+
| ``soft``         | Task that can be violated by the solver, but it                        |
|                  | will try to minimize the error.                                        |
|                  | The relative importance of ``soft`` tasks is defined by their          |
|                  | ``weight``.                                                            |
+------------------+------------------------------------------------------------------------+

Constraints
-----------

Constraints are specific limits that you want your robot to respect. While joint limits and velocity constraints are
implemented in the solver itself, other specific constraints can be added to the solver.

Those can be prioritized and weighted in the exact same way as tasks, as explained above.

Removing tasks and constraints
------------------------------

Tasks and constraints can be removed from the solver by using
:func:`remove_task <placo.DynamicsSolver.remove_task>` and :func:`remove_constraint <placo.DynamicsSolver.remove_constraint>`:

.. code-block:: python

    # Removing a task
    solver.remove_task(some_task)

    # Removing a constraint
    solver.remove_constraint(some_constraint)