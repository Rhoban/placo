Visualization
=============

PlaCo comes with some basics visualization tools based on 
`meshcat <https://github.com/meshcat-dev/meshcat>`_. Those helpers are written in
Python and available in the ``placo_utils`` package.

Command-line visualization
--------------------------

The following utils is provided to visualize a robot:

.. code-block:: bash

    python -m placo_utils.view path_to_robot/robot.urdf

You can provide the following arguments:

* ``--frames [frame1 [frame2 [...]]]``: List of frames to visualize
* ``--animate`` to animate the robot (a sinusoïdal motion is applied to the joints)

Robot visualization
-------------------

You can use :func:`robot_viz <placo_utils.visualization.robot_viz>` to visualize a robot,
and :func:`robot_frame_viz <placo_utils.visualization.robot_frame_viz>` to visualize a frame
attached to the robot.

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/visualization/videos/robot.mp4
        :loop:

    :example:`visualization/robot.py`

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
