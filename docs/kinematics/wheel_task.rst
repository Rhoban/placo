Wheel task
==========

The :func:`WheelTask <placo.WheelTask>` can be used to create a constraint on
a joint that is a wheel of a given radius acting on a given surface (by default, the ground of the world).

Being a wheel means that the instantaneous contact point can't slide in the wheel direction.
A wheel can be omnidirectional (in that case, the contact point can slide laterally) or not
(in that case, the contact point can't slide laterally).

Creating the task
-----------------

Creating a wheel task is done by calling the :func:`add_wheel_task() <placo.KinematicsSolver.add_wheel_task>` method
on the solver:

.. code-block:: python

    # Creating a wheel task for the joint wheel1, with a radius of 0.04m, and that is omnidirectional
    wheel1_task = solver.add_wheel_task("wheel1", 0.04, True)
    wheel1_task.configure("wheel1", "hard")

The arguments of the :func:`add_wheel_task() <placo.KinematicsSolver.add_wheel_task>` method are the
name of the joint, the radius of the wheel, and a boolean indicating if the wheel is omnidirectional or not.

Updating the task
-----------------

The following parameters can be updated on the wheel task:

.. code-block:: python

    # Updating the omniwheel status of the wheel
    wheel1_task.omniwheel = False

    # Updating the radius of the wheel
    wheel1_task.radius = 0.05

    # Updating the surface on which the wheel is acting
    wheel1_task.T_world_surface = tf.translation_matrix([0.0, 0.0, 0.5])

Example
-------

Here is an example of an omnidirectional robot wheel:

.. admonition:: Omniwheel robot
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/omniwheel.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/omniwheel.py`