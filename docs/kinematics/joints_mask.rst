Masking joints
==============

By default, all the joints can be used by the solver, which includes the *floating base*, that is the pose
of the robot in the world. You can decide to mask some joints that will be constrained not to move.

Masking floating base
---------------------

To freeze the floating base, you can use the :func:`mask_fbase() <placo.KinematicsSolver.mask_fbase>` method:

.. code-block:: python

    # Masks the floating base (i.e. the pose of the robot in the world)
    # this should be done for fixed-base robots
    solver.mask_fbase(True)


Masking custom joints
---------------------

For all the other joints, you can use the :func:`mask_dof() <placo.KinematicsSolver.mask_dof>` and
:func:`unmask_dof() <placo.KinematicsSolver.unmask_dof>` methods:

.. code-block:: python

    # The head_yaw will not be allowed to move while solving
    solver.mask_dof("head_yaw")

    # Unmask the head_yaw, it will be allowed to move while solving
    solver.unmask_dof("head_yaw")