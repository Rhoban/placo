Frame tools
===========

Frame yaw
---------

The method :func:`frame_yaw <placo.frame_yaw>` extracts the yaw of a given frame.

Flatten on floor
----------------

Method :func:`flatten_on_floor <placo.flatten_on_floor>` can be used to flatten a frame on the floor.
It takes its ``z`` components, pitch and roll to zero.

Rotation from axis
------------------

The method :func:`rotation_from_axis <placo.rotation_from_axis>` builds a rotation matrix so that the given axis
points towards the given direction. Note that there is an arbitrary choice here, but the result is deterministic.

This can be convenient if you only care a bout a specific axis (for example, a normal surface).

.. admonition:: Rotation from axis example
    
    The following example shows the result of :func:`rotation_from_axis <placo.rotation_from_axis>`, asking
    a rotation matrix to track the purple arrow.
    At each full revolution, the axis is switched periodically between ``x``, ``y`` and ``z``.

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/tools/videos/rotation_from_axis.mp4
        :loop:

    :example:`tools/rotation_from_axis.py`

Optimal transformation
----------------------

For two given set of points, the :func:`optimal_transformation <placo.optimal_transformation>` function can be used to find the optimal transformation between them.

This tool is useful in the case where some points are used to retrieve the corresponding transformation.

.. admonition:: Optimal transformation example
    
    In the example below, the optimal transformation is computed between the red points on the floor and the robot legs,
    allowing to find the optimal match to place the robot on the floor.
    This can be convenient for state estimation.

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/tools/videos/optimal_transformation.mp4
        :loop:

    :example:`tools/optimal_transformation.py`