Handling loop closures
======================

Understanding loop closures
---------------------------

Robots with loop closures are robots containing loops in their kinematics chain.
To understand better this concern, please refer to the `loop closure section <../kinematics/loop_closures>`_
of the kinematics documentation. It will explain you how to model such closures within the URDF format.

Loop closure in dynamics: checklist
-----------------------------------

Here is the checklist to handle loop closures within a dynamics solver:

1. Add tasks to ensure loop closures, just like you would `do for kinematics <../kinematics/loop_closures>`_.
   Those tasks are typically :doc:`relative position tasks <position_orientation_frame_task>`
2. Create a :doc:`task contact <task_contact>` associated with the loop closure task.
   This will add internal forces to the system to enforce the loop closure.
3. Make sure all passive degrees of freedom have zero-torque constraint, thanks to the
   `torque task <torque_task>`_.

Breaking down planar 2DoF example
---------------------------------

Here is an example, that we will break down with details:


.. admonition:: Planar 2DoF

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/planar_2dof.mp4
        :autoplay:
        :muted:
        :loop:

    In this example, a planar 2DoF parallel robot is controlled.
    The loop closure is handled with a relative position task, which is associated with a generic task
    contact.

    :example:`dynamics/planar_2dof.py`

In the source code, you will find the following lines:

.. code-block:: python

    # Adding a relative position task with contact at the effector
    loop_closure = solver.add_relative_position_task(
        "closing_effector_1", "closing_effector_2", np.array([0.0, 0.0, 0.0])
    )
    loop_closure.mask.set_axises("xy")
    loop_closure.configure("closure", "hard")

Here, the loop closure task is created as a constraint of the relative position for ``closing_effector_1`` and ``closing_effector_2``. The ``xy`` mask is added, because there are no way to compute the ``z`` axis of the loop closure. The task is configured as a hard constraint, meaning that the solver will strictly enforce this constraint.

The following line:

.. code-block:: python

    loop_closure_contact = solver.add_task_contact(loop_closure)

Creates a :doc:`task contact <task_contact>` associated with the loop closure task. Thanks to this, forces can be added by the solver to
enforce the task.

Finally, the following lines:

.. code-block:: python

    # Imposing zero torque on passive degrees of freedom
    torque_task = solver.add_torque_task()
    torque_task.set_torque("passive1", 0.0)
    torque_task.set_torque("passive2", 0.0)
    torque_task.configure("torque", "hard")

Ensure that the passive degrees of freedom have zero-torque constraint. This is done with a
:doc:`torque task <torque_task>`, which is configured as a hard constraint.

Examples
--------

With a similar pattern, many loop closures are used un Megabot:

.. admonition:: Megabot

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/megabot.mp4
        :autoplay:
        :muted:
        :loop:

    Megabot is a giant quadrupedal robot using linear actuators.
    Many loop closures are present in its design.

    :example:`dynamics/megabot.py`

In this other example, gear tasks are used with task contact to simulate a differential gear system:

.. admonition:: Differential

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/differential.mp4
        :autoplay:
        :muted:
        :loop:

    A differential gear system.
    At the end of the video, the torque is forced to zero to show the system's behaviour when only subject
    to gravity.

    :example:`dynamics/differential.py`
