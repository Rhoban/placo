Solver status
=============

Dump solver status
------------------

For debugging purpose, you might want to investigate what the solver status is.
To that end, you can call the :func:`dump_status() <placo.KinematicsSolver.dump_status>` on the solver:

.. code-block:: python

    # Shows the current tasks status
    solver.dump_status()

This will display the task names, type, priority and current errors.

For the humanoid example (:example:`kinematics/humanoid.py`), the output will be similar to:

.. code-block:: text

    * Kinematics Tasks:
        * right_foot_orientation [orientation]
            - Priority: soft (weight:1)
            - Error: 0.000012 [rad]

        * right_foot_position [position]
            - Priority: soft (weight:1)
            - Error: 0.000306 [m]

        * reg [joints]
            - Priority: soft (weight:1e-05)
            - Error: 3.216593 [dof]

        * left_foot_orientation [orientation]
            - Priority: soft (weight:1)
            - Error: 0.000029 [rad]

        * trunk_orientation [orientation]
            - Priority: soft (weight:1)
            - Error: 0.000012 [rad]

        * left_foot_position [position]
            - Priority: soft (weight:1)
            - Error: 0.000370 [m]

        * look_ball [axis_align]
            - Priority: soft (weight:1)
            - Error: 0.005295 [rad]

        * com [com]
            - Priority: soft (weight:1)
            - Error: 0.000630 [m]

Accessing tasks errors
----------------------

You can access task error using :func:`error() <placo.KinematicsTask.error>` and :func:`error_norm <placo.KinematicsTask.error_norm>`:

.. code-block:: python

    # Accessing the error of the right foot orientation task
    error = right_foot_orientation.error()
    error_norm = right_foot_orientation.error_norm()
