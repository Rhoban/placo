Hard equalities elimination
===========================

Like the kinematics solver, the dynamics solver can eliminate its hard equalities before calling the QP solver, with
different methods that are a choice left to the user. They are selected with the same flags of ``solver.problem``,
please refer to the `hard equalities elimination section <../kinematics/equalities_elimination>`_ of the kinematics
documentation.

.. note::

    The sparse elimination currently doesn't help with the dynamics solver, even for robots with many loop closures
    like Megabot: the equations of motion of a floating base robot, and the zero-torque constraints on passive joints,
    involve variables of the whole robot, so the equalities can't be split in independent groups, and it falls back
    to the QR elimination.
