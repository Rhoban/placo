CoM polygon constraint
======================

Statically, it is known that keeping the center of mass of a mobile robot within its support polygon
(the convex hull of the contact points) is necessary to prevent the robot from tilting over.

The :class:`CoMPolygonConstraint <placo.CoMPolygonConstraint>` can be used to enforce the (2D) CoM
to stay within a given polygon.

Creating the constraint
-----------------------

Adding this constraint can be done as follows:

.. code-block:: python

    # Support polygon (must be clockwise)
    polygon = np.array([
        [-0.15, 0.],
        [0.02, 0.15],
        [0.02, -0.15]
    ])

    # Adding the constraint, with 1.5 cm margin
    com_constraint = solver.add_com_polygon_constraint(polygon, 0.015)
    com_constraint.configure("com_constraint", "hard")

Here, the polygon is a list of 2D points (on the ground) defining the clockwise support polygon.
Clockwise means that the points are ordered in such a way that the interior of the polygon is on
the right of the line going from one point to the next.

Updating the constraint
-----------------------

The constraint can be updated at any time, by setting the :func:`polygon <placo.CoMPolygonConstraint.polygon>`
and the :func:`margin <placo.CoMPolygonConstraint.margin>` attributes of the constraint:

.. code-block:: python

    # Updating the polygon and margin
    com_constraint.polygon = polygon
    com_constraint.margin = 0.015

Example
-------

The :func:`CoMPolygonConstraint <placo.CoMPolygonConstraint>` is used in the following example, where a quadruped
robot is trying to reach targets while not tilting over:

.. admonition:: Quadruped reaching targets while not tilting
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/quadruped_targets.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/quadruped_targets.py`