Distance task
=============

The :func:`DistanceTask <placo.DistanceTask>` can be used to constrain two points to remain at a given distance.

Creating the constraint
-----------------------

You can use the :func:`add_distance_task() <placo.KinematicsSolver.add_distance_task>` method
on the solver to create a cone constraint:

.. code-block:: python

    # Create a cone constraint between the "base" and "effector" frames
    distance_task = solver.add_distance_task("point1", "point2", 0.1)

This will constraint `point1` and `point2` to remain at `0.1m`.

Updating the constraint
-----------------------

The constraint angle can be updated by setting the :func:`angle_max <placo.DistanceTask.distance>`
property:

.. code-block:: python

    # Update the constraint distance
    distance_task.distance = 0.12

