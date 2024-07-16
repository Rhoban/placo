Puppet contact
==============

*To be used with:* no task required

The puppet contact is an "universal contact", allowing the solver to add arbitrary forces anywhere
on the robot. By essence, it makes all the tasks feasible force-wise.

This contact is helpful for debugging purpose, and can be used in the initialization phase to
set the robot in a specific state.

.. code-block:: python

    # Create the puppet contact
    puppet_contact = solver.add_puppet_contact()

For example, you can consider such a loop at the begining to initialize the robot:

.. code-block:: python
        
    # Initializing the robot using a puppet contact
    puppet_contact = solver.add_puppet_contact()
    for k in range(1000):
        solver.solve(True)
        robot.update_kinematics()
    solver.remove_contact(puppet_contact)

.. admonition:: Quadruped puppet

    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/quadruped_puppet.mp4
        :autoplay:
        :muted:
        :loop:

    The quadruped in this example is achieving flying-like tasks.
    This is made possible by the addition of a "puppet contact", providing arbitrary necessary forces.

    :example:`dynamics/quadruped_puppet.py`
