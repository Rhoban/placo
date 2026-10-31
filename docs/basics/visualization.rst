Visualization
=============

PlaCo comes with some basics visualization tools based on 
`viser <https://viser.studio>`_. Those helpers are written in
Python and available in the ``placo_utils`` package.

The viewer is served by the Python process itself: open the URL it prints (by default
``http://localhost:8080``) in a web browser.

Command-line visualization
--------------------------

The following utils is provided to visualize a robot:

.. code-block:: bash

    python -m placo_utils.view path_to_robot/robot.urdf

You can provide the following arguments:

* ``--frames [frame1 [frame2 [...]]]``: List of frames to visualize (all the frames by default, they can be
  shown or hidden from the scene tree of the viewer)
* ``--animate`` to animate the robot (a sinusoïdal motion is applied to the joints)
* ``--mjcf`` to load a MuJoCo (MJCF) file

The joints can be moved with sliders in the viewer.

Robot visualization
-------------------

You can use :func:`robot_viz <placo_utils.visualization.robot_viz>` to visualize a robot,
and :func:`robot_frame_viz <placo_utils.visualization.robot_frame_viz>` to visualize a frame
attached to the robot.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/robot.mp4
        :loop:

    :example:`visualization/robot.py`

Ground
------

The viewer shows the ground: a grid (10 cm cells, 1 m sections) at :math:`z = 0`. It can be hidden
from the viewer ("Ground" checkbox), or with :func:`ground_viz <placo_utils.visualization.ground_viz>`, which can
also move it to another height:

.. code-block:: python

    # Ground at z = -0.1
    ground_viz(z=-0.1)

    # Hiding the ground
    ground_viz(visible=False)

Custom frame
------------

You can use :func:`frame_viz <placo_utils.visualization.frame_viz>` to visualize any frame
in the world frame, by passing ``T_world_frame`` as argument.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/frame.mp4
        :loop:

    :example:`visualization/frame.py`

Points
------

You can use :func:`point_viz <placo_utils.visualization.point_viz>` to visualize a single point,
or :func:`points_viz <placo_utils.visualization.points_viz>` to visualize a list of points.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/point.mp4
        :loop:

    :example:`visualization/point.py`

Lines
------

You can use :func:`line_viz <placo_utils.visualization.line_viz>` to visualize lines.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/lines.mp4
        :loop:

    :example:`visualization/lines.py`

Arrow
------

You can use :func:`arrow_viz <placo_utils.visualization.arrow_viz>` to visualize an arrow.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/arrow.mp4
        :loop:

    :example:`visualization/arrow.py`

Editing tasks from the viewer
-----------------------------

The targets of position tasks and frame tasks (of the kinematics or dynamics solver) can be edited from the viewer
with :func:`task_controls <placo_utils.visualization.task_controls>`: a gizmo is placed at the target, and dragging
it (or rotating it, for frame tasks) moves the target.

.. code-block:: python

    effector_task = solver.add_frame_task("effector", robot.get_T_world_frame("effector"))
    task_controls(effector_task, "effector")

:func:`position_task_controls <placo_utils.visualization.position_task_controls>` and
:func:`frame_task_controls <placo_utils.visualization.frame_task_controls>` can also be used directly. The targets
are then updated by the viewer, they should not be changed by the code at the same time.

.. admonition:: Example

    :example:`kinematics/6axis_controls.py`

Sliders
-------

You can use :func:`add_slider <placo_utils.visualization.add_slider>` to add a slider to the viewer panel, to change
or tune any value while the program runs (a target, a gain, a task weight...). Its current value is
``slider.value``:

.. code-block:: python

    height = add_slider("CoM height", 0.2, 0.4, 0.3, step=0.005)
    weight = add_slider("posture weight", 0.0, 1.0, 1e-3, step=1e-4)

    # In the loop
    com_task.target_world = np.array([0.0, 0.0, height.value])
    posture_task.configure("posture", "soft", weight.value)

:func:`joints_sliders <placo_utils.visualization.joints_sliders>` adds a slider for each joint of a robot, within its
limits:

.. code-block:: python

    sliders = joints_sliders(robot)

    # In the loop
    for joint, slider in sliders.items():
        robot.set_joint(joint, slider.value)

.. admonition:: Example

    :example:`kinematics/6axis_sliders.py`
