Joints task
===========

A :func:`JointsTask <placo.JointsTask>` is a task that can be used to define a target value for some joints.

.. warning::

    While it is possible to explicitely set a joint value by using :func:`set_joint() <placo.RobotWrapper.set_joint>`
    on the robot wrapper (see :doc:`joints configurations </basics/configurations>`),
    it is **not** equivalent to using a joint task.
    
    Using the joints task will allow to account for everything
    that the solver is enforcing (joint limits, joint velocity limits, other tasks such as loops closing constraints
    etc...), while :func:`set_joint() <placo.RobotWrapper.set_joint>` will immediately update the joint value
    in the robot state regardless of the solver.

Task initialization
-------------------

You can use the following code to initialize the joint task:

.. code-block:: python

    # Initializing a joints task as a soft task with weight 1.0
    joints_task = solver.add_joints_task()
    joints_task.configure("joints", "soft", 1.0)

Task update
-----------

You can configure the task by passing a ``dict`` to :func:`set_joints() <placo.JointsTask.set_joints>`:

.. code-block:: python

    # Setting the target for the joints task
    joints_task.set_joints({
        "joint1": 0.5,
        "joint2": -0.2,
    })

.. note::

    Unit values will depend on the joint types, it will be *radians* for revolute joints and *meters* for prismatic


Used as regularization
----------------------

As explained in :ref:`the concepts <regularization>`, it is possible to use a task with a low weight for
regularization. A joint task can be adapted for that purpose, by using a robot posture as "fallback"
target when the problem is underconstrained.


Example
-------

Here is an example of a 6-axis arm, where it's base joint (``r1``) is controlled by a joint task:

.. admonition:: 6-axis joint
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/6axis_joint.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/6axis_joint.py`