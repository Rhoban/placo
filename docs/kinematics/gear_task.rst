Gear task
=========

A :func:`GearTask <placo.GearTask>` can be used to create a relation between two degrees of freedom.

It can be used to simulate the behaviour of a gear (with a given ratio) or a timing belt, but also of any other
situation where a degree of freedom is mimicking the behaviour of another one.

Creating the task
-----------------

The gear task can be created by calling :func:`add_gear_task() <placo.KinematicsSolver.add_gear_task>` on 
the solver:

.. code-block:: python

    # Creating the gear task
    gear_task = solver.add_gear_task()
    gear_task.configure("gear", "hard")

Setting up the relations
------------------------

Relations can then be set by calling :func:`set_gear() <placo.GearTask.set_gear>` on the task:

.. code-block:: python

    # Setting up the relations
    gear_task.set_gear("joint1", "joint2", 2.0)

    # After that, joint1 = 2 * joint2

The first two arguments are the names of the degrees of freedom that are linked together. The third argument is the
ratio between the two degrees of freedom. You can also call :func:`add_gear() <placo.GearTask.add_gear>` to make
the relation multiple:

.. code-block:: python

    # Adding a gear relation
    gear_task.set_gear("joint1", "joint2", 2.0)
    gear_task.add_gear("joint1", "joint3", 3.0)

    # After that, joint1 = 2 * joint2 + 3 * joint3

Example
-------

In the following example, the planar 2 DoF robot is being used with an additional gear task on its two joints
with a relation of ``-1.0``. One of the degrees of freedom is then moved with a joints task, leading to the other
one moving in the opposite direction.

.. admonition:: Planar 2 DoF (gear relation)
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/planar_2dof_gear.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/planar_2dof_gear.py`

In this other example, gears are coupled with more complex relations:

.. admonition:: Differential
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/differential.mp4
        :autoplay:
        :muted:
        :loop:

    A differential gear system.

    :example:`kinematics/differential.py`