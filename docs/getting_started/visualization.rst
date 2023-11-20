Visualization
=============

PlaCo comes with some basics visualization tools based on 
`meshcat <https://github.com/meshcat-dev/meshcat>`_. Those helpers are written in
Python and available in the ``placo_utils`` package.

Robot visualization
-------------------

You can use :func:`placo_utils.visualization.robot_viz` to visualize a robot,
and :func:`placo_utils.visualization.robot_frame_viz` to visualize a frame
attached to the robot.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/robot.mp4
        :loop:

    :example:`visualization/visualize_robot.py`

Custom frame
------------

You can use :func:`placo_utils.visualization.frame_viz` to visualize any frame
in the world frame, by passing ``T_world_frame`` as argument.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/frame.mp4
        :loop:

    :example:`visualization/visualize_frame.py`

Points
------

You can use :func:`placo_utils.visualization.point_viz` to visualize a point.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/point.mp4
        :loop:

    :example:`visualization/visualize_point.py`

Lines
------

You can use :func:`placo_utils.visualization.line_viz` to visualize lines.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/lines.mp4
        :loop:

    :example:`visualization/visualize_lines.py`

Arrow
------

You can use :func:`placo_utils.visualization.arrow_viz` to visualize an arrow.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/arrow.mp4
        :loop:

    :example:`visualization/visualize_arrow.py`
