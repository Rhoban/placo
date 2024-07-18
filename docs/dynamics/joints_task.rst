Joints task
===========

A :func:`JointsTask <placo.DynamicsJointsTask>` is a task that can be used to define a target value for some joints.

.. warning::

    While it is possible to explicitely set a joint value by using :func:`set_joint() <placo.RobotWrapper.set_joint>`
    on the robot wrapper (see :doc:`joints configurations </basics/configurations>`),
    it is **not** equivalent to using a joint task.
    
    Using the joints task will allow to account for everything
    that the solver is enforcing (torque limits, joint limits, joint velocity limits, other tasks such as loops
    closing constraints etc...), while :func:`set_joint() <placo.RobotWrapper.set_joint>` will immediately update
    the joint value in the robot state regardless of the solver.

Task initialization
-------------------

You can use the following code to initialize the joint task:

.. code-block:: python

    # Initializing a joints task as a soft task with weight 1.0
    joints_task = solver.add_joints_task()
    joints_task.configure("joints", "soft", 1.0)

Task update
-----------

You can configure the task by passing a ``dict`` to :func:`set_joints() <placo.DynamicsJointsTask.set_joints>`:

.. code-block:: python

    # Setting the target for the joints task
    joints_task.set_joints({
        "joint1": 0.5,
        "joint2": -0.2,
    })

.. note::

    Unit values will depend on the joint types, it will be *radians* for revolute joints and *meters* for prismatic

To specify the target velocity and acceleration, you can call :func:`set_joints() <placo.DynamicsJointsTask.set_joint>`
on individual joints:

.. code-block:: python

    # Setting target position, velocity, and acceleration
    joints_task.set_joint("shoulder_pan_joint", np.sin(t), np.cos(t), -np.sin(t))

Example
-------

Here is a complete example using a UR5 robot tracking a target frame.
Its target velocity is also set for better tracking.

.. admonition:: UR5 controlled with joints
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/ur5_joints.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, a fixed contact is used on the base of the UR5 robot, which is following a sinusoidal
    joint-space trajectory.

    :example:`dynamics/ur5_joints.py`