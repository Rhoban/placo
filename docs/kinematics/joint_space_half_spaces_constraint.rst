Joint-space Half-spaces constraint
==================================

Sometime, the workspace of a robot can be bound by a simple box, using :doc:`joint limits <joint_limits>`.

However, for more complex robots where self-collisions are present, it can be considered to use a more complex set of inequalities, bounding the robot joint-space in a polytope of the form:

.. math::

    A q \le b

Where :math:`A` is a :math:`n_c \times n_q` matrix, and :math:`b` a :math:`n_c` vector. :math:`n_c` being the number of constraints and :math:`n_q` the dimension of the robot :doc:`configuration space (state.q) </basics/configurations>`.

Each row of such a constraint is actually an half-space in the robot configuration space. This is what :func:`joint-space half-spaces constraint <placo.JointSpaceHalfSpacesConstraint>` does.

Creating the constraint
-----------------------


.. code-block:: python

    # Initializing A and b for one constraint        
    nq = len(robot.state.q)
    A = np.zeros((1, nq))
    b = np.zeros(1)

    # We want r1 + r2 <= 1
    A[0, robot.get_joint_offset("r1")] = 1
    A[0, robot.get_joint_offset("r2")] = 1
    b[0] = 1

    # Adding the constraint
    hspace_hspaces_constraint = solver.add_joint_space_half_spaces_constraint(A, b)
    hspace_hspaces_constraint.configure("workspace", "hard")

You first need to create the matrix :math:`A` and the vector :math:`b` of proper sizes. Creating the constraint is then straightforward.

.. note::

    Even if :math:`A` has the same number of columns as the robot :math:`q` vector for convenience, the floating base terms are ignored by :func:`JointSpaceHalfSpacesConstraint <placo.JointSpaceHalfSpacesConstraint>`

Updating the constraint
-----------------------

The polytope can be updated by setting :func:`A <placo.JointSpaceHalfSpacesConstraint.A>` and :func:`b <placo.JointSpaceHalfSpacesConstraint.b>`

.. code-block:: python

    # Updating the polytope
    hspace_hspaces_constraint.A = new_A
    hspace_hspaces_constraint.b = new_b
