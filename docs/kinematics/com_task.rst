CoM task
========

The :func:`CoMTask <placo.CoMTask>` is a task that can be used to impose a target position for the center of mass
in the world.

.. note::
    Because every body contributes to the center of mass, this task can have unexpected behaviors when used on
    under-constrained formulations. We encourage you using a :ref:`regularization task <regularization>`
    in conjunction with this task.

Task initialization
-------------------

The CoM task can be initialized by calling :func:`add_com_task() <placo.KinematicsSolver.add_com_task>` on the
solver:

.. code-block:: python

    # Initialize the CoM task
    com_task = solver.add_com_task(np.array([0., 0., 0.3]))
    com_task.configure("com", "soft", 1.0)

Update the task
---------------

The task can be updated by setting :func:`target_world <placo.CoMTask.target_world>`:

.. code-block:: python

    # Update the target position
    com_task.target_world = np.array([0., 0., 0.3])

Example
-------

The CoM task is used in the following example to swing the robot body laterally:

.. admonition:: Humanoid
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/humanoid.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/humanoid.py`