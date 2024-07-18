Handling loop closures
======================

For some robots, the kinematics chain can't be represented as a tree because of loop closures.
For instance, the *6-axis* robot has no loop closure since it is a **serial robot**, but the 
*planar 2 dof* robot has a loop closure since it is a **parallel robot**:

.. |6axis| image:: https://github.com/Rhoban/placo-examples/blob/master/models/6axis/robot.png?raw=true
   :width: 400px
   :alt: 6 axis robot

.. |planar2dof| image:: https://github.com/Rhoban/placo-examples/blob/master/models/planar-2dof/robot.png?raw=true
   :width: 400px
   :alt: planar 2 dof robot

+------------------+------------------+
| 6 axis robot     | planar 2 dof     |
| (no loop closure)| (loop closure)   |
+==================+==================+
| |6axis|          | |planar2dof|     |
+------------------+------------------+

URDF representation
-------------------

Parallel robots can't be represented as-is in the URDF format, since it only supports tree structures.
The common way to represent them is to "break" the parallel robot as a tree robot, and attach some frames
where the loop closures are.

For instance, the planar 2 dof robot can be represented as two separate branches, both ending with two frames:

.. image:: https://github.com/Rhoban/placo-examples/blob/master/models/planar-2dof/opened_chain.png?raw=true
    :width: 400px
    :alt: planar 2 dof robot opened

Loop closure constraint
-----------------------

A task can then be defined to enforce the loop closure constraint. In the above example of the planar 2 dof robot,
if we call ``closing_effector_1`` the frame attached to the first branch, and ``closing_effector_2`` the frame
attached to the second branch, we can use a :func:`RelativePositionTask <placo.RelativePositionTask>` to enforce
this closure:

.. code-block:: python

    # Loop closure task
    closing_task = solver.add_relative_position_task("closing_effector_1", "closing_effector_2", np.zeros(3))
    closing_task.configure("closing", "hard", 1.0)
    closing_task.mask.set_axises("xy")

Here:

* The ``np.zeros(3)`` vector is the desired position of the ``closing_effector_1`` frame in the ``closing_effector_2`` frame.
* We use a ``hard`` constraint, meaning that the solver really has to strictly enforce this constraint to be true
  (we don't want a solution that would break the loop closure).
* We use a ``mask`` to only enforce the constraint on the ``xy`` axises. Since the closure is enforced in a plane.

.. note::

    Depending on the robot configuration, any other task could be used as a loop closure constraint.

Moving joints in a parallel robot
---------------------------------

In serial robots, forward kinematics is straightforward computation from the joints configuration. 
However, for a parallel robot, the loop closure constraints need to be enforced all the times.
To do that you might also want to use a solver with a :doc:`joints task <joints_task>` to move the joints
while enforcing the loop closure constraint. 
Note that your initial configuration will affect the result, since this is a numerical solution and not an analytical one.

.. note::

    If needed, it is possible to create multiple solvers for the same robot, each with a different set of tasks
    (e.g.: one for joint control and the other for task-space control).

If you want to do both state estimation and control, you might then want to instantiate multiple kinematic solver
with different tasks sets. In the :doc:`examples gallery <examples_gallery>`, you can find both joint and task
control for the *planar 2 dof* and *3-axis parallel rotation* robots.


Example
-------

Here are two examples depicting the use of the solver with the planar 2 DoF for both joints and task control:

.. admonition:: Planar 2 DoF (joints control)
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/planar_2dof_joints.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/planar_2dof_joints.py`

.. admonition:: Planar 2 DoF (trajectory control)
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/planar_2dof_trajectory.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/planar_2dof_trajectory.py`