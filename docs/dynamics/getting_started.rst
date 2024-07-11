Getting started
===============

In this section, we provide an example to take a first step using the dynamics solver.

Creating the dynamics solver
----------------------------

Let's use the `UR5 <https://github.com/Rhoban/placo-examples/tree/master/models#6axis-ur5>`_ model from
the examples:

.. code-block:: python

    import placo

    # Loading the UR5 robot modle
    robot = placo.RobotWrapper("models/ur5")
    # Building a dynamics solver
    solver = placo.DynamicsSolver(robot)
    # Setting the solver delta time to 1ms
    solver.dt = 0.001

The ``solver`` will be the tool we'll use to formulate our tasks, contacts and constraints.

Defining tasks
--------------

Our robot is a static arm, we can start by disabling the floating base:

.. code-block:: python

    # Disabling the floating base
    solver.mask_fbase(True)

Now, we will add a frame task for the effector:


.. code-block:: python

    # Adding a frame task to control the "ee_link" frame
    effector_task = solver.add_frame_task("ee_link", np.eye(4))
    # Configuring the task as soft, with the same weight for position and orientation
    effector_task.configure("ee_link", "soft", 1.0, 1.0)

The ``np.eye(4)`` is the initial target pose for the task. We will update it later.

Enabling limits
---------------

We will now enable all possible limits, since the UR5 URDF file is specifying properly the robot abilities:

.. code-block:: python

    # Torque limits (enabled by default)
    solver.enable_torque_limits(True)
    # Joints and velocity limits (disabled by default)
    solver.enable_velocity_limits(True)
    solver.enable_joint_limits(True)

Updating the task
-----------------

Let's update the task by updating a t ime ``t``, and setting :func:`T_world_frame <placo.DynamicsFrameTask.T_world_frame>` on the ``effector_task`` accordingly:

.. code-block:: python

    effector_task.T_world_frame = tf.translation_matrix([0.4, 0.25 * np.sin(t), 0.3])

Running the solver
------------------

Running the solver can be done by calling :func:`solve() <placo.DynamicsSolver.solve>`:

.. code-block:: python

    # Solving the inverse dynamics problem
    # The True flag indicate that we want the robot state to be updated accordingly
    result = solver.solve(True)

The :class:`result <placo.DynamicsSolverResult>` object returned by :func:`solve() <placo.DynamicsSolver.solve>`
provides the computed acceleration, torques, and contact torques:

.. code-block:: python

    if result.success:
        print("Tau:", result.tau)
        print("Tau contacts: ", result.tau_contacts)
        print("Acceleration: ", result.qdd)

Passing ``True`` to :func:`solve() <placo.DynamicsSolver.solve>` means that the robot state will be updated
by an integration of the computed acceleration.

Putting it all together, you'll get an example similar to this:

.. admonition:: UR5 tracking targets without velocity tracking
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/ur5_targets_no_velocity.mp4
        :autoplay:
        :muted:
        :loop:

    Note: to reproduce the above example, pass ``--no-velocity`` argument when running the example

    :example:`dynamics/ur5_targets.py`


Specifying target velocity
--------------------------

As you can see on the previous example, the effector is not properly tracking the target. This is because we
only use a feedback controller, with an implicit target velocity set to zero.
You can think of it as the effector trying to reach the target with no velocity, meaning it is getting slower and
slower as it approaches the target.

To account for that, you can specify target velocities while tracking the target, let's turn our velocity into a
form that allow us to do so:

.. code-block:: python
        
    def get_trajectory(t: float):
        # Target effector pose (4x4 matrix)
        T_world_effector = tf.translation_matrix([0.4, 0.25 * np.sin(t), 0.3])
        # Target effector position velocity
        dtarget_world = np.array([0.0, 0.25 * np.cos(t), 0.0])

        return T_world_effector, dtarget_world

And change our task update as follow:

.. code-block:: python

    T_world_effector, dtarget_world = get_trajectory(t)
    effector_task.T_world_frame = T_world_effector
    effector_task.position().dtarget_world = dtarget_world

You should get something similar to this:

.. admonition:: UR5 tracking targets, with velocity tracking
    
    .. video:: https://github.com/Rhoban/placo-examples/raw/master/dynamics/videos/ur5_targets.mp4
        :autoplay:
        :muted:
        :loop:

    :example:`dynamics/ur5_targets.py`