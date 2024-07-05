Getting started 
===============

Introduction
------------

The kinematics solver allows you to formulate *task-space inverse kinematics* problems, including constraints
such as joint limits and velocities, through a convenient API. This page will guide you through the main
concepts through an introduction example.

Creating the kinematics solver
------------------------------

The :func:`KinematicsSolver <placo.KinematicsSolver>` class is the main entry point to build the inverse kinematics
problem. It has to be initialized with the :func:`RobotWrapper <placo.RobotWrapper>` class as first argument,
which wraps all the :doc:`robot model </basics/load_robots>` methods:

.. code-block:: python

    import placo

    # Loading a robot
    robot = placo.RobotWrapper('models/6axis/')
    # Initializing the kinematics solver
    solver = placo.KinematicsSolver(robot)

Since our 6-axis robot is not mobile, we will disable the *floating base* (the robot body is not allowed to move
in the world, it is "anchored to the ground"):

.. code-block:: python

    # The floating base can't move
    solver.mask_fbase(True)

As you can see in other examples, this *floating base* is necessary for mobile robot to move in the world.

Adding tasks
------------

Let's assume we want to take the robot *effector* to a given position and orientation. For that, we can
add a :func:`FrameTask <placo.FrameTask>` to the solver:

.. code-block:: python

    # Adding a frame task
    effector_task = solver.add_frame_task("effector", np.eye(4))
    effector_task.configure("effector", "soft", 1.0, 1.0)

Here:

* ``effector`` is the name of the frame (as present in the URDF file) we want to control,
* ``np.eye(4)`` is the desired pose (we initialise it to identity because we will update it later)
* ``configure()`` is a method allowing us to configure the task in the solver
    * ``effector`` is the name of the task
    * ``soft`` means that the task is not hard-constrained (the solver will do its best to reach the desired pose,
      but will not fail if it's not reachable)
    * the two last arguments are the weight of the position and orientation tasks.

Updating the task
-----------------

The ``effector_task`` we created lives in the solver and will be used for subsequent computations. We can
update the target pose:

.. code-block:: python

    from placo_utils.tf import tf
    # Updating the target pose of the effector task
    effector_task.T_world_frame = tf.translation_matrix([1.25, np.sin(t), 1.0])

Running the solver !
--------------------

To run the solver, we first need to ensure that the robot underlying quantities of interrest are up to
date, by calling :func:`update_kinematics() <placo.RobotWrapper.update_kinematics>` on the robot, and
then run the solver by calling :func:`solve() <placo.KinematicsSolver.solve>`:

.. code-block:: python
    
    # Updating kinematics computations (frames, jacobians, etc.)
    robot.update_kinematics()
    # Solving the IK
    solver.solve(True)

The boolean argument to :func:`solve() <placo.KinematicsSolver.solve>` means that we want to reflect the
solution in the robot state by integrating it (you can think of it as :math:`q` being updated to :math:`q + \Delta q`).

.. note::

    PlaCo is often used in a loop fashion, in that case, we recommend the following pattern:

    .. code-block:: python
        
        # Be sure
        robot.update_kinematics()

        while is_running: # some main loop
            # Update tasks data here

            # Solve the IK
            solver.solve(True)

            # Update frames and jacobians
            robot.update_kinematics()

            # Optionally: dump the solver status
            solver.dump_status()


Putting it all together
-----------------------

Putting all the above parts together and adding some visualization will result in the following example:

.. admonition:: 6-axis basic
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/kinematics/videos/6axis_basic.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`kinematics/6axis_basic.py`

You can find more examples in the :doc:`examples gallery <examples_gallery>`.

See also
--------

* `Pink <https://github.com/stephane-caron/pink>`_, a task-space inverse kinematics library based on Pinocchio (Python)
