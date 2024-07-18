Torque task
===========

The :func:`DynamicsTorqueTask <placo.DynamicsTorqueTask>` is a task that can be used to impose a target torque.
This is useful to represent joints that are passive, like a bearing, a spring or a dampener.

Task initialization
-------------------

The torque task can be initialized by calling :func:`add_com_task() <placo.DynamicsSolver.add_torque_task>` on the
solver, which takes no argument:

.. code-block:: python

    # Creating a torque task
    torque_task = solver.add_torque_task()

Task update
-----------

The task can be updated by calling :func:`set_torque() <placo.DynamicsTorqueTask.set_torque>`:

.. code-block:: python

    # Setting the torque to 0.0
    torque_task.set_torque("left_knee", 0.0)

    # Don't forget that the task is actually soft by default, like the others
    # If you want to enforce strictly zero torque, make the task hard
    torque_task.configure("torque", "hard")

Alternatively, extra parameters `kp` and `kd` can be passed to the `set_torque` method:

.. code-block:: python

    # Setting the torque to 0.0 with a stiffness of 1.0 and a damping of 0.1
    # The joint will act as some kind of passive spring
    torque_task.set_torque("left_knee", 0.0, 1.0, 0.1)

    # Setting the torque to 0.0 with a stiffness of 0.0 and a damping of 0.1
    # The joint will have zero torque, and act as a dampener
    torque_task.set_torque("left_knee", 0.0, 0.0, 0.1)

To stop applying torque, you can call the :func:`reset_torque() <placo.DynamicsTorqueTask.reset_torque>` method:

.. code-block:: python

    # Reset the torque (stop acting on the joint)
    torque_task.reset_torque("left_knee")

Example
-------

.. admonition:: Humanoid with external force applied

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/sigmaban_external_wrench.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, a torque task is used to set the motors torque to zero.
    In the mean time, an external contact is applied on the robot's foot.

    :example:`dynamics/sigmaban_external_wrench.py`

.. admonition:: Megabot

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/megabot.mp4
        :autoplay:
        :muted:
        :loop:

    In this robot, many passive joints are used. The torque is enforced to zero by a torque task.

    :example:`dynamics/megabot.py`