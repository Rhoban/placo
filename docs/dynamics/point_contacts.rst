Point contacts
==============

Fixed point contact
-------------------

*To be used with:* :func:`position task <placo.DynamicsPositionTask>`

This type of contact can be used for points that are attached to the world.

.. code-block:: python

    # Creating a leg task
    leg_task = robot.add_position_task("leg1")
    # Creating a fixed point contact
    # Warning: this is NOT an unilateral contact, it can pull on the ground
    leg_contact = robot.add_fixed_point_contact(leg_task)

.. admonition:: Hanging quadruped

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/quadruped_hanging.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, a quadruped robot is attached with four points (which are not unilateral).
    All motors except one are unactuated (torque is set to zero).
    This results in a "hanging" robot.

    :example:`dynamics/quadruped_hanging.py`

Point (unilateral) contact
--------------------------

*To be used with:* :func:`position task <placo.DynamicsPositionTask>`

.. code-block:: python

    # Creating a leg task
    leg_task = robot.add_position_task("leg1")
    # Creating an unilateral point contact
    leg_contact = robot.add_point_contact(leg_task)
    # If needed, the normal to the surface in the world can be set
    leg_contact.R_world_surface = np.eye(3)
    # You can adjust the friction coefficient by setting mu
    leg_contact.mu = 0.5

.. _quadruped_example:

.. admonition:: Quadruped

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/quadruped.mp4
        :autoplay:
        :muted:
        :loop:

    A quadruped robot is on the floor with 4 unilateral contact points.

    :example:`dynamics/quadruped.py`
