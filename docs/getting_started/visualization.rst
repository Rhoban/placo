Visualization
=============

PlaCo comes with some basics visualization tools based on 
`meshcat <https://github.com/meshcat-dev/meshcat>`_. Those helpers are written in
Python and available in the ``placo_utils`` package.

Visualize robot
---------------

You can import :func:`placo_utils.visualization.robot_viz` to visualize a robot:

.. code-block:: python
        
    import placo
    import time
    from placo_utils.visualization import robot_viz

    robot = placo.RobotWrapper("quadruped/robot.urdf")
    viz = robot_viz(robot)

    while True:
        viz.display(robot.state.q)
        time.sleep(1.0)

Visualizing robot frame
-----------------------

If you want to visualize any frame attached to the robot, import
:func:`placo_utils.visualization.robot_frame_viz`:

.. code-block:: python

    from placo_utils.visualization import robot_frame_viz

You can then call:

.. code-block:: python

    # In the visualization loop
    robot_frame_viz(robot, "body")

Visualizing custom frame
------------------------

You can use :func:`placo_utils.visualization.frame_viz` to visualize any frame
in the world frame, by passing ``T_world_frame`` as argument:

.. code-block:: python

    from placo_utils.visualization import frame_viz, robot_frame_viz
    import numpy as np

    # Visualizing a custom target
    T_world_target = compute_target()
    frame_viz("target", T_world_target)

    # Visualizing the effector position
    robot_frame_viz(robot, "effector")

