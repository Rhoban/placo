Cone constraint
===============

The :func:`ConeConstraint <placo.ConeConstraint>` can be used to constrain two frames z-axises to be
aligned within a certain angle. 

Creating the constraint
-----------------------

You can use the :func:`add_cone_constraint() <placo.KinematicsSolver.add_cone_constraint>` method
on the solver to create a cone constraint:

.. code-block:: python

    # Create a cone constraint between the "base" and "effector" frames
    cone_constraint = solver.add_cone_constraint("base", "effector", 0.4)
    cone_constraint.configure("cone", "hard")

This will constrain the z-axis of the ``base`` frame to be within 0.4 radians of the z-axis of the
``effector`` frame.

Updating the constraint
-----------------------

The constraint angle can be updated by setting the :func:`angle_max <placo.ConeConstraint.angle_max>`
property:

.. code-block:: python

    # Update the constraint angle
    cone_constraint.angle_max = 0.5

Example
-------

In the following example, a cone constraint is used to restrict the effector of a parallel
3-axis robot to a cone around the base frame:

.. admonition:: Orbita 3-axis parallel rotation (task control)
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/orbita_task.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/orbita_task.py`