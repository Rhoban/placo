Regularization
==============

In the case of underconstrained tasks, there might be many solutions to the problem: a choice has to be
made about the preferred solution.
Regularization is the addition of a very low cost to the objective function, in order to favour some solutions
over others.
Do not hesitate to have a look at :ref:`kinematics regularization <regularization>`.

Torque regularization
---------------------

In the dynamics solver, the torque can be expressed as the other decision variables.
The (squared) torque is regularized by default, meaning that :math:`\sum \tau_i^2` will be minimized,
with a default weight of :math:`10^{-3}`.
This weight can be adjusted with :func:`solver.torque_cost <placo.DynamicsSolver.torque_cost>`:

.. code-block:: python

    # Weight for the torque regularization (1e-3 by default)
    solver.torque_cost = 1e-2

Pose regularization
-------------------

**(WORK IN PROGRESS)**

