External wrench contact
=======================

*To be used with:* no task required

An external wrench contact can be used to apply an external force and/or moments to a given frame.

.. code-block:: python

    # Adding an external wrench, which will be expressed in the world
    # and applied on  the right foot
    external_wrench = solver.add_external_wrench_contact("right_foot", "world")
    
    # Setting the wrench (force, moments)
    exexternal_wrench.w_ext = np.array([10, 0, 0, 0, 0, 0])

.. admonition:: Humanoid with external force applied

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/sigmaban_external_wrench.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, the humanoid robot has a fixed based and zero torque in the motors.
    An external wrench is applied on its right foot.

    :example:`dynamics/sigmaban_external_wrench.py`

