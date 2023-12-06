Joint configurations
====================

Robot state
-----------

The robot configuration is stored in the ``RobotWrapper`` class, in the
:func:`state <placo.RobotWrapper.state>` members:

* ``robot.state.q`` for :math:`q` the joint positions, you can use the helpers:
    * :func:`get_joint <placo.RobotWrapper.get_joint>`: to retrieve the joint value of a given joint name,
    * :func:`set_joint <placo.RobotWrapper.set_joint>`: to set the joint value of a given joint name,
* ``robot.state.qd`` for :math:`\dot{q}` the joint velocities, you can use the helpers:
    * :func:`get_joint_velocity <placo.RobotWrapper.get_joint_velocity>`: to retrieve the joint velocity of a given joint name,
    * :func:`set_joint_velocity <placo.RobotWrapper.set_joint_velocity>`: to set the joint velocity of a given joint name.
* ``robot.state.qdd`` for :math:`\ddot{q}` the joint accelerations, you can use the helpers:
    * :func:`get_joint_acceleration <placo.RobotWrapper.get_joint_acceleration>`: to retrieve the joint acceleration of a given joint name,
    * :func:`set_joint_acceleration <placo.RobotWrapper.set_joint_acceleration>`: to set the joint acceleration of a given joint name.

For example:

.. code-block:: python

    # Sets head_pan joint to 0.5 rad
    robot.set_joint("head_pan", 0.5)

Integrating
-----------

You can call :func:`integrate <placo.RobotWrapper.integrate>` to integrate the robot state. This will integrate the
acceleration, velocity and position of the robot for a given ``dt``.

Joint limits
------------

Limits are loaded by default from the URDF file. However, you can override the joint limits
by using :func:`set_joint_limits <placo.RobotWrapper.set_joint_limits>`:

.. code-block:: python

    # Sets head_pan joint limits to [-1.0, 1.0] rad
    robot.set_joint_limits("head_pan", -1.0, 1.0)

You can also update the velocity and torque limits, by using respectively
:func:`set_velocity_limit <placo.RobotWrapper.set_velocity_limit>` and
:func:`set_torque_limit <placo.RobotWrapper.set_torque_limit>`.