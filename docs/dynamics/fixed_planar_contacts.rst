Fixed and planar contacts
=========================

Fixing the floating base
------------------------

Assuming a robot is fixed (no floating base), you can use the :func:`mask_fbase() <placo.DynamicsSolver.mask_fbase>`
method to disable the floating base:

.. code-block:: python

    # Disable the floating base
    solver.mask_fbase(True)

This will add a constraint on the floating base that should have no acceleration, and allow forces to be applied
by the floating base to compensate for bias forces such as gravity.

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

