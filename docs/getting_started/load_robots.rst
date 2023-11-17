Loading robots
==============

Loading your Robot
------------------

To load a robot, you'll need to use the :func:`RobotWrapper <placo.RobotWrapper>` class. This class is wrapping
the robot model, state and exposing all convenient methods to interact with your robot model.

Using URDF file
---------------

To load your robot, you can simply specify an URDF file this way:

.. code-block:: python
    
    import placo
    robot = placo.RobotWrapper("model/robot.urdf")

.. note::
    If you only specify a directory, the ``robot.urdf`` file will be loaded.

.. _robot_wrapper_flags:

Flags
-----

The second argument of :func:`RobotWrapper <placo.RobotWrapper>` is a flag to specify modifiers. Available
flags are the following:

+-------------------------------------+----------------------------------------------------+
| Flag                                | Description                                        |
+=====================================+====================================================+
| ``placo.Flags.collision_as_visual`` | Load collision geometry as visual geometry.        |
+-------------------------------------+----------------------------------------------------+
| ``placo.Flags.ignore_collisions``   | Ignore all collisions (remove all the pairs).      |
+-------------------------------------+----------------------------------------------------+

For more information about the handling of self-collisions, see :doc:`collisions`. An example
would be:

.. code-block:: python

    import placo
    # Loading a robot, ignoring all collisions
    robot = placo.RobotWrapper("model/robot.urdf", placo.Flags.ignore_collisions)

Using direct URDF contents
--------------------------

The third argument of :func:`RobotWrapper <placo.RobotWrapper>` can be provided if you want to
give the contents of the URDF file directly:

.. code-block:: python

    import placo
    # Providing directly the URDF contents
    robot = placo.RobotWrapper("model/", 0, urdf_contents)


