Avoid self collisions constraint
================================

Once your :doc:`collisions </basics/collisions>` are setup, the
:func:`AvoidSelfCollisionsConstraint <placo.AvoidSelfCollisionsConstraint>` can be used to
prevent self collisions between the robot links.

Internally, it uses the :ref:`distances computation <collision_distances>` to compute the distances and
closest points between possible collision pairs. An inequality is then added during the solve step to
prevent the closest points to be closer than a given margin.

.. warning::

    While this feature might be useful for some applications, you should be aware of the following
    points:

    * This constraint can add significant computation time if you have many links that can collide.
      Be sure your :doc:`collision model </basics/collisions>` is clean, prefferably with pure shapes and
      avoiding complex shapes.
    * Be sure your shapes don't overlap. Otherwise, please specify your collision pairs
      in the :doc:`collision model </basics/collisions>`.
    * The kinematics solver only solves for one small timestep ahead.
      For this reason, self collisions avoidance could lead a trajectory to a local optimum where the
      robot is stuck. Getting out of such situation is a planning problem which is out of the scope of
      such solver.

Creating the constraint
-----------------------

The constraint can be created by calling :func:`add_avoid_self_collisions_constraint() <placo.KinematicsSolver.add_avoid_self_collisions_constraint>` on the solver:

.. code-block:: python

    # Adds the constraint to the solver
    collisions_constraint = solver.add_avoid_self_collisions_constraint()

Configuring the constraint
--------------------------

You can configure the following parameters on the constraint:

.. code-block:: python

    # Margin to avoid self collisions
    collisions_constraint.self_collisions_margin = 0.01 # 1cm

    # Distance where the constraint starts being active in the solver
    # (to avoid extra computations for parts that are far away)
    collisions_constraint.self_collisions_trigger = 0.05 # 5cm

Example
-------

.. admonition:: Humanoid robot with self collisions avoidance
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/humanoid_collisions.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/humanoid_collisions.py`