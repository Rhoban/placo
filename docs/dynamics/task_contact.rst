Task contact
============

*To be used with:* any task

As explained in the introduction, any task can be associated to a contact, allowing forces to be added.
Whether this is physically meaningful or not is up to the user.
A rule of thumb is that if the task is not handled by the physics, it should be associated with a contact.

Intuitively, associating a task with a contact provides free force that will ensure the task is achieved,
regardless of the robot's ability to do so.
For example, a loop closure task, that is in practice ensured by the integrity of a bearing,
can be associated with a contact.

.. code-block:: python

    # From the example below, the loop closure tasks asks for the effector
    # frames to be at the same position
    loop_closure = solver.add_relative_position_task(
        "closing_effector_1", "closing_effector_2", np.array([0.0, 0.0, 0.0])
    )
    # The generic task contact is added, this will allow forces to be added to ensure this
    # task.
    loop_closure_contact = solver.add_task_contact(loop_closure)

.. admonition:: Planar 2DoF

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/planar_2dof.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, a planar 2DoF parallel robot is controlled.
    The loop closure is handled with a relative position task, which is associated with a generic task
    contact.

    :example:`dynamics/planar_2dof.py`


