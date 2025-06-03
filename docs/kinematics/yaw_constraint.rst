Yaw constraint
===============

The :func:`YawConstraint <placo.YawConstraint>` can be used to constrain the yaw of the x-axis of frame B with respect to the z-axis of frame A.

Creating the constraint
-----------------------

You can use the :func:`add_yaw_constraint() <placo.KinematicsSolver.add_cone_add_yaw_constraint>` method
on the solver to create a yaw constraint:

.. code-block:: python

    # Creating a yaw constraint between the base and the effector frames
    # Effector x-axis must have a yaw within 0.4 radians of the base z-axis of frame B
    yaw_constraint = solver.add_yaw_constraint("base", "effector", 0.4)
    yaw_constraint.configure("yaw", "hard")

This will constrain the x-axis of the ``effector`` frame to be within 0.4 radians of the z-axis of the
``base`` frame.

Updating the constraint
-----------------------

The constraint angle can be updated by setting the :func:`angle_max <placo.YawConstraint.angle_max>`
property:

.. code-block:: python

    # Update the constraint angle
    yaw_constraint.angle_max = 0.5

Example
-------

In the following example, a spherical parallel manipulator is rotated with a yaw constraint enforcing it to stay within 0.4 radians of the base frame's z-axis:

.. admonition:: Orbita 3-axis parallel rotation (yaw constraint)
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/orbita_yaw.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/orbita_yaw.py`