Interpolation
=============

Frames interpolation
--------------------

The :func:`interpolate_frames <placo.interpolate_frames>` function can be used to interpolate between two frames.

.. code-block:: python

    # Interpolating between frame1 and frame2
    # The last argument (AtoB) is the interpolation factor: 0: frameA, 1: frameB
    T_world_interpolated = placo.interpolate_frames(T_world_frameA, T_world_frameB, 0.2)

.. admonition:: Example
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/tools/videos/interpolate_frames.mp4
        :loop:

    :example:`tools/interpolate_frames.py`


Cubic splines
-------------

The :class:`CubicSpline <placo.CubicSpline>` class can be used to create a point to point trajectory,
while imposing velocities.

.. code-block:: python

    spline = placo.CubicSpline()

    # Arguments are time, position and velocity
    spline.add_point(0.0, 0.0, 0.0)
    spline.add_point(1.0, 3.0, 0.0)
    spline.add_point(2.0, -2.0, 0.0)
    spline.add_point(3.0, 0.0, 0.0)

    # Retrieving the position, velocity and acceleration at time t with:
    t = 0.5
    pos = spline.pos(t) 
    vel = spline.vel(t)
    acc = spline.acc(t)

:example:`tools/cubic_spline.py`.

3D Cubic Splines
----------------

The :class:`CubicSpline3D <placo.CubicSpline3D>` class can be used to create a point to point trajectory in 3D.
It basically wraps 3 :class:`CubicSpline <placo.CubicSpline>` objects together.

.. code-block:: python

    spline = placo.CubicSpline3D()

    # The API is similar to CubicSpline, with a convenient wrapper of dimension 3
    spline.add_point(0., np.array([0., 0., 0.]), np.array([0., 0., 0.]))
    spline.add_point(1., np.array([3., 0., 0.]), np.array([0., 0., 0.]))
    spline.add_point(2., np.array([0., 3., 0.]), np.array([0., 0., 0.]))
    spline.add_point(3., np.array([0., 0., 3.]), np.array([0., 0., 0.]))

    # Positions, velocities and acceleration will return 3D vectors
    pos = spline.pos(0.5) 