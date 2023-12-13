Axis-align task
===============

The :func:`AxisAlignTask <placo.AxisAlignTask>` aligns an axis expressed in a robot frame to another
axis expressed in the world.

Creating the task
-----------------

The axis align task can be created by calling :func:`add_axisalign_task() <placo.KinematicsSolver.add_axisalign_task>`
on the solver:

.. code-block:: python

    # Aligns the camera z-axis with the world x-axis
    axisalign_task = solver.add_axisalign_task("camera", np.array([0., 0., 1.]), np.array([1., 0., 0.]))
    axisalign_task.configure("look_at", "soft", 1.0)

Updating the task
-----------------

The target axis can be updated by setting the :func:`targetAxis_world <placo.AxisAlignTask.targetAxis_world>`:

.. code-block:: python

    # Update the task so that the world axis match the camera to ball vector
    axisalign_task.targetAxis_world = ball - camera_pos

Example
-------

In the following example, an humanoid robot is looking at a ball by using the axis align task:

.. admonition:: Humanoid robot
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/humanoid.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/humanoid.py`