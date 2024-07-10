Introduction
============

The dynamics solver allows you to formulate *task-space inverse dynamics* problems, including constraints
such as velocity, joint limits and torque limits.

Differences with the kinematics solver
--------------------------------------

In the :doc:`Kinematics Solver </kinematics/getting_started>` (which we encourage you to get familiar with in
first place), a variation of the robot configuration :math:`\Delta q` is computed to achieve desired tasks.
To that end, a first-order approximation of the motion is made, allowing for linear equations to be used.

In the dynamics solver, an acceleration :math:`\ddot q` is computed to achieve desired tasks.
Thanks to the equation of motion, the associated torque :math:`tau` can be expressed as a function of
the acceleration and used in expressions.

1) Tasks are expressed as desired acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A typical task structure will look like:

:math:`\ddot q^{desired} = K_p (q^{desired} - q) + K_d (\dot q^{desired} - \dot q) + \ddot q^{desired}`.

Where the desired position, velocity and accelerations are given by the user.
If you only have a position reference, the desired velocity and acceleration can be set to zero.
In that case, :math:`K_p` can be adjusted to change the stiffness, and :math:`K_d` can be set to
:math:`2 \sqrt{K_p}` to have a `critically damped system <https://en.wikipedia.org/wiki/Damping>`_.
This is the default in the solver to reduce the number of parameters to tune.

2) Tasks and constraints can refer to torque
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The good news in the dynamics solver, is that the equation of motion is enforced!

It allows to express the torque :math:`\tau` as a function of the acceleration and robot state (see below for
details if you are interrested).
In practice, this allows to use torque in constraint and tasks.
The very first natural constraints are torque limits, accounting better for the robot abilities.


3) Contact needs to be specified
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In the case of a mobile robot, the universal *floating base* is not actuated.
This implies that the torque applied is constrained to be zero.
Because of that, the only way to get the robot moving is to interact with its environment.
In a kinematics solver, there is no causal relation between the robot and the environment forces,
while in the dynamics solver, the contacts **must** be properly specified in order to produce the desired motion.

To that end, the dynamics solver allow you to specify active contacts, that are used in the computation.


.. admonition:: Math details

    The solver enforces the following equation:

    :math:`\tau + \sum_i J_i(q)^T F_i = M(q) \ddot q + h(q, \dot q)`,

    where :math:`\tau` is the torque, :math:`Ji(q)` is the Jacobian of the :math:`i`-th contact,
    :math:`F_i` is the external force for the :math:`i`-th contact, :math:`M(q)` is the mass matrix,
    and :math:`h(q, \dot q)` is the bias term (lumping gravity, centrifugal and coriolis).

    The decision variables are :math:`\ddot q` and the contact forces :math:`F_i`.