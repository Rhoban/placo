Distance constraint
===============

The :func:`DistanceConstraint <placo.DistanceConstraint>` can be used to constrain the distance between two points to be lower than a given value.

Creating the constraint
-----------------------

You can use the :func:`add_distance_constraint() <placo.KinematicsSolver.add_cone_add_distance_constraint>` method
on the solver to create a distance constraint:

.. code-block:: python

    # Creating a distance constraint, constraining the right foot to remain within 0.3 meters of the trunk
    distance_constraint = solver.add_distance_constraint("trunk", "right_foot", 0.3)
    distance_constraint.configure("distance", "hard")

This will constrain the distance between the "trunk" and "right_foot" frames to be less than 0.3 meters.

Updating the constraint
-----------------------

The constraint distance can be updated by setting the :func:`distance_max <placo.YawConstraint.distance_max>`
property:

.. code-block:: python

    # Update the constraint distance
    distance_constraint.distance_max = 0.5

Example
-------

In the following example, an humanoid robot moves its right foot, which is constrained to remain within 0.3 meters of the trunk.

.. admonition:: Humanoid robot with distance constraint
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/humanoid_distance.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/humanoid_distance.py`