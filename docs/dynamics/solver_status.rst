Solver status
=============

Dumping the status
------------------

For debugging purpose, you might want to investigate what the solver status is.
To that end, you can call the :func:`dump_status() <placo.DynamicsSolver.dump_status>` on the solver:

.. code-block:: python

    # Shows the current tasks status
    solver.dump_status()

This will display the task names, type, priority and current errors.

For the humanoid example (:example:`dynamics/sigmaban.py`), the output will be similar to:

.. code-block:: text

    * Dynamics Tasks:
        * posture [joints]
            - Priority: soft (weight:1e-06)
            - Error: 1.636099 [dof]
            - DError: 0.434214 [dof]

        * rightFoot_orientation [orientation]
            - Priority: hard
            - Error: 0.000000 [rad]
            - DError: 0.000000 [rad]

        * com [com]
            - Priority: soft (weight:1)
            - Error: 0.002318 [m]
            - DError: 0.037381 [m]

        * leftFoot_orientation [orientation]
            - Priority: hard
            - Error: 0.000000 [rad]
            - DError: 0.000000 [rad]

        * trunk [orientation]
            - Priority: soft (weight:1)
            - Error: 0.000001 [rad]
            - DError: 0.000000 [rad]

        * rightFoot_position [position]
            - Priority: hard
            - Error: 0.000000 [m]
            - DError: 0.000005 [m]

        * leftFoot_position [position]
            - Priority: hard
            - Error: 0.000000 [m]
            - DError: 0.000005 [m]

Accessing tasks errors
----------------------

You can access task error using :func:`error <placo.DynamicsTask.error>` and :func:`derror <placo.DynamicsTask.derror>`:

.. code-block:: python

    effector_position_task = solver.add_position_task("effector", np.array([0.0, 0.0, 0.0]))

    ... 

    # Returns the position error
    effector_position_task.error

    # Returns the velocity error
    effector_position_task.derror