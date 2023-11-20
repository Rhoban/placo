Self-collisions
===============

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

Using self-collisions
---------------------

Checking for self-collisions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By calling :func:`placo.RobotWrapper.self_collisions`, you can get the current self collisions
as a list of :func:`placo.Collision` objects.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/robot_wrapper/videos/collisions.mp4
        :loop:

    :example:`robot_wrapper/collisions.py`

Checking for nearest distances
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By calling :func:`placo.RobotWrapper.distances`, you can get all the current minimum distances
between the collision pairs as a list of :func:`placo.Distance` objects.

See also
--------

When using the kinematics solver, :func:`placo.KinematicsSolver.add_avoid_self_collisions_constraint` can
be called to add a constraint to the solver so that self collisions are prevented.