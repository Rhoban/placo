Hard equalities elimination
===========================

Tasks with ``hard`` priority are equality constraints of the underlying QP problem (see
:doc:`tasks and constraints <concepts>`). Before calling the QP solver, PlaCo can *eliminate* those equalities:
it computes all the solutions that satisfy them, and then only searches among those solutions, with less variables
and without equality constraints.

This is a choice left to the user, since the fastest method depends on the problem. All the methods give the same
solution (up to numerical rounding), only the computation time changes.

Available methods
-----------------

The method is selected with two flags of the solver's underlying problem, ``solver.problem``. The same flags
are available for the kinematics and dynamics solvers:

.. code-block:: python

    # QR elimination (default)
    solver.problem.rewrite_equalities = True
    solver.problem.sparse_elimination = False

    # No elimination: the equalities are passed to the QP solver
    solver.problem.rewrite_equalities = False

    # Sparse elimination
    solver.problem.rewrite_equalities = True
    solver.problem.sparse_elimination = True

+------------------------+----------------------------------------------------------------------------------+
| Method                 | Description                                                                      |
+========================+==================================================================================+
| QR elimination         | The hard equalities are eliminated with a QR decomposition of all of them.       |
| (default)              | This is a robust general choice.                                                 |
+------------------------+----------------------------------------------------------------------------------+
| No elimination         | The hard equalities are passed as-is to the QP solver, which handles them as     |
|                        | constraints.                                                                     |
+------------------------+----------------------------------------------------------------------------------+
| Sparse elimination     | The equalities are split in small independent groups, each involving few         |
|                        | variables (typically, one group per loop closure), eliminated separately.        |
|                        | When the equalities can't be split this way, it falls back to QR elimination.    |
+------------------------+----------------------------------------------------------------------------------+

Which one should I use?
-----------------------

* **QR elimination** is the default and a good choice in most cases.
* **Sparse elimination** is interesting for robots with several independent
  :doc:`loop closures <loop_closures>` (for instance, a closed chain in each leg). The cost of the QR elimination
  grows with the size of the whole robot, while the cost of the sparse elimination grows with the size of each loop.
  It doesn't help when there are few hard equalities, or when they all involve the same joints (for instance a
  single closure spanning the whole robot). Since it falls back to QR elimination, it is safe to enable.
  It currently only helps with the kinematics solver: with the dynamics solver, the equations of motion and the
  zero-torque constraints involve variables of the whole robot, and it falls back to QR elimination.
* **No elimination** is faster on some problems, for instance some of the dynamics examples below, and much
  slower on others.

Here are some solve times measured on the examples:

+-------------------------------------------+--------+----------------+-----------+
| Example                                   | QR     | No elimination | Sparse    |
+===========================================+========+================+===========+
| Kinematics, Megabot (many loop closures)  | 52 µs  | 55 µs          | **16 µs** |
+-------------------------------------------+--------+----------------+-----------+
| Kinematics, humanoid                      | 8 µs   | 8 µs           | 8 µs      |
+-------------------------------------------+--------+----------------+-----------+
| Dynamics, Sigmaban                        | 167 µs | **126 µs**     | 169 µs    |
+-------------------------------------------+--------+----------------+-----------+
| Dynamics, quadruped                       | 44 µs  | **31 µs**      | 43 µs     |
+-------------------------------------------+--------+----------------+-----------+
| Dynamics, Megabot (fallback to QR)        | 405 µs | 985 µs         | 406 µs    |
+-------------------------------------------+--------+----------------+-----------+

.. note::

    The best way to choose is to measure the solve time on your own robot and tasks.

Inspecting the elimination
--------------------------

After a solve, the following attributes of ``solver.problem`` can be checked:

* ``n_equalities``: the number of equality constraints,
* ``free_variables``: the number of variables that remained after the elimination,
* ``determined_variables``: the number of variables that were determined by the equalities,
* ``sparse_elimination_used``: whether the sparse elimination was actually used (else it fell back to the QR
  elimination).

.. code-block:: python

    solver.problem.sparse_elimination = True
    solver.solve(True)

    if not solver.problem.sparse_elimination_used:
        print("The hard equalities could not be split, QR elimination was used")
