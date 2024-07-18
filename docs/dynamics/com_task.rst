CoM task
========

The :func:`CoMTask <placo.DynamicsCoMTask>` is a task that can be used to impose a target position, velocity
and/or acceleration for the center of mass in the world.

.. note::
    Because every body contributes to the center of mass, this task can have unexpected behaviors when used on
    under-constrained formulations. We encourage you using a :ref:`regularization task <regularization>`
    in conjunction with this task.

Task initialization
-------------------

The CoM task can be initialized by calling :func:`add_com_task() <placo.DynamicsSolver.add_com_task>` on the
solver:

.. code-block:: python

    # Initialize the CoM task
    com_task = solver.add_com_task(np.array([0., 0., 0.3]))
    com_task.configure("com", "soft", 1.0)

Task update
-----------

The task can be updated by setting :func:`target_world <placo.CoMTask.target_world>`:

.. code-block:: python

    # Update the target position
    com_task.target_world = np.array([0., 0., 0.3]) # target position
    com_task.dtarget_world = np.array([0., 0., 0.]) # target velocity (optional)
    com_task.ddtarget_world = np.array([0., 0., 0.]) # target acceleration (optional)

Example
-------

.. _sigmaban_example:

.. admonition:: Sigmaban humanoid

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/sigmaban.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, the robot's center of mass is moved from side to side.

    :example:`dynamics/sigmaban.py`