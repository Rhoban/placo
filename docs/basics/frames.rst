Frames
======

.. important::
    Everytime the robot internal state changes, you have to call the method
    :func:`update_kinematics <placo.RobotWrapper.update_kinematics>` to be sure that the frames are up to date.

Frame to world transformation
-----------------------------

The transformation 4x4 matrix from a given ``frame`` to the world can be obtained with
:func:`get_T_world_frame <placo.RobotWrapper.get_T_world_frame>`:

.. code-block:: python

    # Retrieve the body to world transformation (4x4) matrix
    T_world_body = robot.get_T_world_frame('body')

Moving the robot in the world
-----------------------------

You can move the robot in the world by using :func:`set_T_world_frame <placo.RobotWrapper.set_T_world_frame>`. This
will actually update the *floating base* (i.e the root joint) of the robot accordingly.

.. code-block:: python

    # Move the robot in the world (left foot frame will be at the origin)
    robot.set_T_world_frame('left_foot', np.eye(4))

Frame to frame transformation
-----------------------------

The transformation 4x4 matrix from a given ``frame_b`` to another ``frame_a`` can be obtained with
:func:`get_T_a_b <placo.RobotWrapper.get_T_a_b>`:

.. code-block:: python

    # Retrieve the transformation matrix from the left foot to the right foot
    T_rightFoot_leftFoot = robot.get_T_a_b('right_foot', 'left_foot')