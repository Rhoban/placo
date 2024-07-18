Gear task
=========

A :func:`GearTask <placo.DynamicsGearTask>` can be used to create a relation between two degrees of freedom.

It can be used to simulate the behaviour of a gear (with a given ratio) or a timing belt, but also of any other
situation where a degree of freedom is mimicking the behaviour of another one.

Creating the task
-----------------

The gear task can be created by calling :func:`add_gear_task() <placo.DynamicsSolver.add_gear_task>` on 
the solver:

.. code-block:: python

    # Creating the gear task
    gear_task = solver.add_gear_task()
    gear_task.configure("gear", "hard")

.. warning::

    Since a gear task is actually a contact, you also might want to

    * Add a :doc:`task contact <task_contact>` attached to the gear task
    * Set the :doc:`torque <torque_task>` of passive gears to zero

    Refer to the example below for a detailed example.

Setting up the relations
------------------------

Relations can then be set by calling :func:`set_gear() <placo.DynamicsGearTask.set_gear>` on the task:

.. code-block:: python

    # Setting up the relations
    gear_task.set_gear("joint1", "joint2", 2.0)

    # After that, joint1 = 2 * joint2

The first two arguments are the names of the degrees of freedom that are linked together. The third argument is the
ratio between the two degrees of freedom. You can also call :func:`add_gear() <placo.DynamicsGearTask.add_gear>` to make
the relation multiple:

.. code-block:: python

    # Adding a gear relation
    gear_task.set_gear("joint1", "joint2", 2.0)
    gear_task.add_gear("joint1", "joint3", 3.0)

    # After that, joint1 = 2 * joint2 + 3 * joint3

Example
-------

Here is an example with differential gears:

.. admonition:: Differential
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/differential.mp4
        :autoplay:
        :muted:
        :loop:

    A differential gear system.
    At the end of the video, the torque is forced to zero to show the system's behaviour when only subject
    to gravity.

    :example:`dynamics/differential.py`