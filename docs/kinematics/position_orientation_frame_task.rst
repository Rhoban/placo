Position, orientation and frame tasks
=====================================

:func:`PositionTask <placo.PositionTask>` and :func:`OrientationTask <placo.OrientationTask>` can be respectively
use to control the position and orientation of a particular frame on the robot.
The :func:`FrameTask <placo.FrameTask>` is a combination of both, wrapping the two tasks in a single one.

Position task
-------------

Position tasks can be initialized and updated this way:

.. code-block:: python

    # Creating a position task, arguments are the frame name and goal (world) position
    effector_position = solver.add_position_task("effector", np.array([0., 0., 0.]))
    effector_position.configure("effector_position", "soft", 1.0)

    # Updating the effector target in the world
    effector_position.target_world = np.array([0.1, -0.5, 0.])

Orientation task
----------------

Orientation tasks can be initialized and updated this way:

.. code-block:: python

    # Creating an orientation task, argumanets are the frame name and goal (world) orientation (rotation matrix)
    effector_orientation = solver.add_orientation_task("effector", np.eye(3))
    effector_orientation.configure("effector_orientation", "soft", 1.0)

    # Updating the effector orientation in the world
    effector_orientation.R_world_frame = np.eye(3)

Frame task
----------

Frame task are lumping together a position and an orientation task, and can be initialized this way:

.. code-block:: python

    # Creating a frame task, arguments are the frame name and goal (world) pose (transformation matrix)
    effector_frame = solver.add_frame_task("effector", np.eye(4))
    # Configuring the frame task, the two weights are for the position and orientation tasks respectively
    effector_frame.configure("effector_frame", "soft", 1.0, 1.0)

    # Updating the effector target in the world (transformation matrix)
    # Internally, this will update the position and orientation tasks
    effector_frame.T_world_frame = np.eye(4)

The underlying position and orientation tasks can be accessed with the :func:`position() <placo.FrameTask.position>`
and :func:`orientation() <placo.FrameTask.orientation>` methods:

.. code-block:: python

    effector_frame.position() # Access the position task
    effector_frame.orientation() # Access the orientation task

Relative position and orientation tasks
---------------------------------------

The above mentionned tasks also exists in a *relative* version, where two frames have to be specified.

.. code-block:: python

    # Relative position
    camera_task = solver.add_relative_position_task("trunk", "camera", np.array([0., 0., 0.5]))
    # Setting the target (here, for the camera position in the trunk)
    camera_task.target = np.array([0., 0., 0.4])

.. code-block:: python

    # Relative orientation
    camera_task = solver.add_relative_orientation_task("trunk", "camera", np.eye(3))
    # Setting the target (here, for the camera to trunk rotation)
    camera_task.R_a_b = np.eye(3)

.. code-block:: python

    # Relative frame
    camera_task = solver.add_relative_frame_task("trunk", "camera", np.eye(4))
    # Setting the target (here, for the camera to trunk transformation)
    camera_task.T_a_b = np.eye(4)

Masking axises
--------------

In some case, you only want to assign a task for one or two axises. To that end, you can use the
:func:`axises mask <placo.AxisesMask>` for position and orientation tasks:

.. code-block:: python

    # The position task will only affect the z-axis (x and y will be ignored)
    effector_position.mask.set_axises("z")

By default, this masking will occur in the "task" frame (the world frame for absolute tasks, and the first frame for
relative tasks). Youc can set the second argument of :func:`set_axises() <placo.AxisesMask.set_axises>` to
``"local"`` to enforce the masking to happen in the local frame.

Alternatively, you can also specify ``"custom"`` as the second argument, and provide a custom rotation matrix to
specify the axises in which the task will be applied in the :func:`R_local_world <placo.AxisesMask.R_local_world>`
attribute of the ``mask``.

Example
-------

Here is an example of a 6-axis robot following a target trajectory, expressed as a :func:`FrameTask <placo.FrameTask>`:

.. admonition:: 6-axis trajectory
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/6axis_trajectory.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/6axis_trajectory.py`