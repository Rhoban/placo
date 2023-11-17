Self-collisions
===============

In PlaCo, self-collisions can be involved in several ways:

* By using :func:`placo.RobotWrapper.self_collisions`, you can check if the
  robot is in self-collision. By doing that, you can check if the robot is self-colliding,
  but also retrieve the list of self-colliding pairs of bodies.
* By using :func:`placo.RobotWrapper.distances`, you can compute the distances
  between the robot bodies. This function gives you the closest points between
  each pair of bodies.
* When using the solvers, you can use methods to avoid self collisions, like:
    * :func:`placo.KinematicsSolver.add_avoid_self_collisions_constraint`
    * :func:`placo.DynamicsSolver.add_avoid_self_collisions_constraint`

Configuring self-collision pairs
--------------------------------

By default, all collision pairs will be enabled when loading the robot.

Disabling all collisions
~~~~~~~~~~~~~~~~~~~~~~~~

By passing the :func:`placo.Flags.ignore_collisions` flag when loading the robot
(see :ref:`robot_wrapper_flags`), you can disable all collisions.

Specifying collision pairs
~~~~~~~~~~~~~~~~~~~~~~~~~~

If you want to specify which collision pairs you want to be enabled, you can create a
``collisions.json`` file next to your ``.urdf`` file, with the following structure:

.. code-block:: json

    [
        ["body1", "body2"],
        ["body3", "body4"]
    ]

Where ``body1``, ``body2``, ``body3`` and ``body4`` are the names of the bodies you want
to enable collisions for. Those bodies should be the name of the links in the URDF file.

.. note::
    Instead of using link name (strings), you can also use integers representing geometry
    objects (as the ones returned in :func:`placo.Collision`). Thanks to this, it is possible
    to write a script that samples robot configuration and auto-generates the ``collisions.json``
    file.