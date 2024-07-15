Contacts
========

.. admonition:: What are contacts ?

    Contacts model the constraints imposed by the **interaction with physical constraints**.
    For example, the fact that a foot can't penetrate the ground, or can't (easily) slide on it is a contact.

    In some sense, contacts are **tasks that are taken care of by the physics**, instead of the robot.
    To do so, the environment applies additional forces to the robot.

In PlaCo, any task that is actually handled by physical constraints can be associated with a contact.
To do so, the solver will add **extra forces** as decision variables.

However, physical constraints sometime don't handle all the aspects of a task.
Even if a foot can't penetrate the ground, it can still lift up, no force will prevent it
(this is an unilateral contact).
Also, because of the friction coefficient, the ground can't prevent the foot from sliding, this
depends on the relation between the normal force and thet tangential force.
For those reasons, some specific contacts are implemented in PlaCo.

Fixed contact
-------------

*To be used with:* :func:`frame task <placo.DynamicsFrameTask>`

A fixed contact can be associated with a frame task, and will prevent the frame from moving.

.. code-block:: python

    # Create the base task
    base_task = solver.add_frame_task("base", np.eye(4))
    base_task.configure("base", "hard", 1.0, 1.0)
    # Adds a fixed contact for this task
    base_contact = solver.add_fixed_contact(base_task)

.. admonition:: UR5 with fixed contact
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/ur5_fixed_contact.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, a fixed contact is used on the base of the UR5 robot.
    The :func:`contacts_viz <placo_utils.visualization.contacts_viz>` helper is used to visualize the contacts.

    :example:`dynamics/ur5_fixed_contact.py`

Planar (unilateral) contact
---------------------------

*To be used with:* :func:`frame task <placo.DynamicsFrameTask>`

.. code-block:: python

    # Create the left foot task
    left_foot_task = solver.add_frame_task("left_foot", np.eye(4))
    left_foot_task.configure("left_foot", "hard", 1.0, 1.0)
    # Create the unilateral contact
    left_foot_contact = solver.add_planar_contact(left_foot_task)
    # Adjusting the contact rectangular size, expressed in the local frame
    # length is the contact dimension along x axis
    # width is the contact dimension along y axis
    left_foot_contact.length = 0.1 
    left_foot_contact.width = 0.05
    # Adjusting the friction coefficient (default is 1.0)
    left_foot_contact.mu = 0.5


.. _sigmaban_example:

.. admonition:: Sigmaban humanoid

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/sigmaban.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, planar contacts are used to model the feet contacts of the Sigmaban humanoid.

    :example:`dynamics/sigmaban.py`


Fixed point contact
-------------

*To be used with:* :func:`position task <placo.DynamicsPositionTask>`

This type of contact can be used for points that are attached to the world.

.. admonition:: Hanging quadruped

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/quadruped_hanging.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, a quadruped robot is attached with four points (which are not unilateral).
    All motors expect one are unactuated (torque is set to zero).
    This results in a "hanging" robot.

    :example:`dynamics/quadruped_hanging.py`

Point (unilateral) contact
--------------------------

*To be used with:* :func:`position task <placo.DynamicsPositionTask>`

Puppet contact
--------------

*To be used with:* no task required

External wrench contact
-----------------------

*To be used with:* no task required

Generic task contact
--------------------

*To be used with:* any task

Fixing the floating base
------------------------

Assuming a robot is fixed (no floating base), you can use the :func:`mask_fbase() <placo.DynamicsSolver.mask_fbase>`
method to disable the floating base:

.. code-block:: python

    # Disable the floating base
    solver.mask_fbase(True)

This will add a constraint on the floating base that should have no acceleration, and allow forces to be applied
by the floating base to compensate for bias forces such as gravity.


Activating/desactivating or removing contacts
---------------------------------------------

Contacts can change during the simulation. You can activate or deactivate a contact by setting the
:func:`active <placo.Contact.active>` attribute:

.. code-block:: python

    # At initialization
    right_contact = solver.add_planar_contact(rightFoot_task)

    ...

    # During execution, contact can be activated or deactivated
    right_contact.active = False

.. admonition:: Example

    In the above :ref:`Sigmaban example <sigmaban_example>`, the right foot contact is disabled when the foot

You can also remove a contact by using :func:`remove_contact <placo.DynamicsSolver.remove_contact>`:

.. code-block:: python

    # Remove the right foot contact
    solver.remove_contact(right_contact)

Weighting contact
-----------------

By default, forces and moments generated by the solver has no cost (they are however internally weighted by
the problem solver to make the problem feasible).

If you want to mitigate the forces produced by a specific contact, you can use the :func:`weight_forces <placo.Contact.weight_forces>` attribute.
For planar and fixed contacts, you also can use the :func:`weight_moments <placo.Contact.weight_moments>` attribute.

.. admonition:: Example

    In the above :ref:`Sigmaban example <sigmaban_example>`,  the moments are weighted
    using :func:`weight_moments <placo.Contact.weight_moments>` to encourage the center of pressure to be at the
    center of the foot.