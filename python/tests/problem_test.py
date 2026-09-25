import unittest
import placo
import numpy as np


class TestProblem(unittest.TestCase):
    def assertNumpyEqual(self, a, b, msg=None, epsilon=1e-6):
        if msg is None:
            msg = f"Checking that {a} equals {b}"
        self.assertTrue(np.linalg.norm(a - b) < epsilon, msg=msg)

    def test_expression_arithmetics(self):
        problem = placo.Problem()
        x = problem.add_variable(16)

        e = x.expr()
        self.assertNumpyEqual(e.A, np.eye(16))
        self.assertNumpyEqual((-e).A, -np.eye(16))
        self.assertNumpyEqual(e.b, np.zeros(16))

        # Checking sums and multiplications
        self.assertNumpyEqual((e + e).A, 2 * np.eye(16))
        self.assertNumpyEqual((e - e).A, np.zeros((16, 16)))
        self.assertNumpyEqual((e * 2).A, 2 * np.eye(16))
        self.assertNumpyEqual((2 * e).A, 2 * np.eye(16))

        # Checking sum
        self.assertNumpyEqual((e + np.ones(16)).b, np.ones(16))
        self.assertNumpyEqual((e - np.ones(16)).b, -np.ones(16))
        # This one doesn't work because of how the overload is done
        # self.assertTrue(np.linalg.norm((np.ones(16) + e).b - np.ones(16)) < 1e-6)

        # Checking multiplication
        self.assertNumpyEqual(e.left_multiply(np.eye(16) * 2).A, 2 * np.eye(16))

    def test_expression_constants(self):
        """
        Operations between multi-rows expressions and constants (scalars, vectors), sum and mean
        """
        problem = placo.Problem()
        x = problem.add_variable(4)
        e = x.expr(0, 3)  # 3 rows, 4 columns
        v = np.array([1.0, 2.0, 3.0])

        # Scalars apply to all rows
        self.assertNumpyEqual((e + 1.0).b, np.ones(3))
        self.assertNumpyEqual((e - 1.0).b, -np.ones(3))
        self.assertNumpyEqual((e - 1.0).A, e.A)
        self.assertNumpyEqual((1.0 + e).b, np.ones(3))
        self.assertNumpyEqual((1.0 - e).b, np.ones(3))
        self.assertNumpyEqual((1.0 - e).A, -e.A)

        # Vector - expression (numpy would handle v - e itself, so the C++ operator is called directly)
        self.assertNumpyEqual(e.__rsub__(v).A, -e.A)
        self.assertNumpyEqual(e.__rsub__(v).b, v)
        self.assertNumpyEqual(e.__radd__(v).A, e.A)
        self.assertNumpyEqual(e.__radd__(v).b, v)

        # Sum and mean are over the rows
        self.assertNumpyEqual((e + v).sum().A, np.array([1.0, 1.0, 1.0, 0.0]))
        self.assertNumpyEqual((e + v).sum().b, 6.0)
        self.assertNumpyEqual((e + v).mean().A, np.array([1.0, 1.0, 1.0, 0.0]) / 3)
        self.assertNumpyEqual((e + v).mean().b, 2.0)

        # Solving with such expressions
        problem.add_constraint(x.expr() - 1.0 == 0)
        problem.solve()
        self.assertNumpyEqual(x.value, np.ones(4))

    def test_expressions(self):
        """
        Testing basic expression shapes
        """
        problem = placo.Problem()
        x = problem.add_variable(2)
        y = problem.add_variable(2)

        self.assertEqual(x.expr().A.shape[0], 2)
        self.assertEqual(x.expr(0).A.shape[0], 2)
        self.assertEqual(x.expr(0, 2).A.shape[0], 2)
        self.assertEqual(x.expr(0, 1).A.shape, (2,))
        self.assertEqual(y.expr().A.shape[0], 2)
        self.assertEqual(y.expr(0).A.shape[0], 2)
        self.assertEqual(y.expr(0, 2).A.shape[0], 2)
        self.assertEqual(y.expr(0, 1).A.shape, (4,))

    def test_stacking(self):
        """
        The operator / allow to stack expressions
        """
        problem = placo.Problem()

        x = problem.add_variable(8)
        e = x.expr(0, 1) / x.expr(2, 1) / x.expr(4, 1) / x.expr(6, 1)

        A = np.zeros((4, 8))
        A[0, 0] = 1
        A[1, 2] = 1
        A[2, 4] = 1
        A[3, 6] = 1
        self.assertNumpyEqual(A, e.A, msg="Expected matrix obtained by stacking")

        b = np.zeros(4)
        self.assertNumpyEqual(b, e.b, msg="Expected vector obtained by stacking")

    def test_simple_solve(self):
        problem = placo.Problem()

        # A problem where the sum of all the 16 variables should be equal to 1
        x = problem.add_variable(16)
        problem.add_constraint(x.expr().sum() == np.array([1.0]))
        problem.solve()
        self.assertNumpyEqual(
            x.value,
            1 / 16.0,
            msg="16 values which sum equals 1 should be minimized to 1/16",
        )

        # We add an inequality so that the 0th value should be greater than 2
        problem.add_constraint(x.expr(0, 1) >= np.array([2.0]))
        problem.add_constraint(x.expr(0, 1) <= np.array([10.0]))
        problem.solve()
        self.assertGreaterEqual(x.value[0], 2.0 - 1e-6, msg=f"The value should be >= 2")
        self.assertNumpyEqual(
            x.value[1:], -1 / 15.0, msg=f"The remaining values should be -1/15."
        )

    def test_expression_constraint(self):
        problem = placo.Problem()
        p1 = problem.add_variable(2)
        p2 = problem.add_variable(2)

        # We want P1 to be at 17 / 22
        problem.add_constraint(p1.expr() == np.array([17.0, 22.0]))

        # We want to keep P1 and P2 with a difference not greater than 3, 3
        problem.add_limit(p1.expr() - p2.expr(), np.array([3.0, 3.0]))

        # We impose p2 to be at least 18
        problem.add_constraint(p2.expr(0, 1) >= 18)

        problem.solve()

        self.assertNumpyEqual(
            p1.value, np.array([17, 22]), msg="P1 should be in 17, 22"
        )
        self.assertNumpyEqual(
            p2.value, np.array([18, 19]), msg="P2 should be in 18, 19"
        )

    def test_integrator_matrix(self):
        M = placo.Integrator.upper_shift_matrix(3)

        expected = np.zeros((4, 4))
        expected[:3] = np.eye(4)[1:]

        self.assertTrue((M == expected).all(), msg="Checking system matrix or order 3")

        problem = placo.Problem()
        x = problem.add_variable(32)
        integrator = placo.Integrator(x, np.array([0.0, 0.0, 0.0]), 3, 0.1)

        expected_A = np.array([[1.0, 0.1, 0.005], [0.0, 1.0, 0.1], [0.0, 0.0, 1.0]])
        expected_B = np.array([1 / 6 * 0.1**3, 0.005, 0.1])

        self.assertNumpyEqual(
            integrator.M, expected, msg="Checking system matrix M or order 3"
        )
        self.assertNumpyEqual(
            integrator.A, expected_A, msg="Checking system matrix A or order 3"
        )
        self.assertNumpyEqual(
            integrator.B, expected_B, msg="Checking system matrix B or order 3"
        )

    def test_integrator(self):
        # Creating a problem
        problem = placo.Problem()
        x = problem.add_variable(10)
        integrator = placo.Integrator(x, np.array([1.0, 2.0, 3.0]), 3, 0.1)

        # Adding constraint at arrival
        problem.add_constraint(integrator.expr(10, 0) == 4.0)
        problem.add_constraint(integrator.expr(10, 1) == 5.0)
        problem.add_constraint(integrator.expr(10, 2) == 6.0)

        problem.add_constraint(integrator.expr(5, 0) <= -5.0)

        problem.solve()

        # Testing initial values
        self.assertNumpyEqual(integrator.value(0, 0), 1)
        self.assertNumpyEqual(integrator.value(0, 1), 2)
        self.assertNumpyEqual(integrator.value(0, 2), 3)

        # Testing final values
        self.assertNumpyEqual(integrator.value(1.0, 0), 4)
        self.assertNumpyEqual(integrator.value(1.0, 1), 5)
        self.assertNumpyEqual(integrator.value(1.0, 2), 6)

        # Testing that inequality is still enforced
        self.assertLessEqual(integrator.value(0.5, 0) - 1e-8, -5.0)

    def test_integrator_expr_x0(self):
        # Creating a problem
        problem = placo.Problem()
        x = problem.add_variable(10)
        integrator = placo.Integrator(x, np.array([1.0, 2.0]), 2, 0.1)

        problem.add_constraint(integrator.expr(10, 0) == 2.0)
        problem.add_constraint(integrator.expr(10, 1) == 3.0)

        y = problem.add_variable(10)
        integrator2 = placo.Integrator(y, integrator.expr(10), 2, 0.1)

        problem.add_constraint(integrator2.expr(10, 0) == 0.0)
        problem.add_constraint(integrator2.expr(10, 1) == 0.0)

        problem.solve()

        # Testing the first integrator
        self.assertNumpyEqual(integrator.value(1.0, 0), 2.0)
        self.assertNumpyEqual(integrator.value(1.0, 1), 3.0)

        # Testing that the beginning of the second integrator is the end of the first
        self.assertNumpyEqual(integrator2.value(0.0, 0), 2.0)
        self.assertNumpyEqual(integrator2.value(0.0, 1), 3.0)

        # Testing the second integrator
        self.assertNumpyEqual(integrator2.value(1.0, 0), 0.0)
        self.assertNumpyEqual(integrator2.value(1.0, 1), 0.0)

    def test_soft_inequality(self):
        problem = placo.Problem()
        x = problem.add_variable(1)

        inequality = problem.add_constraint(x.expr() >= 5.0)
        inequality.configure("soft", 1.0)

        inequality = problem.add_constraint(x.expr() >= 6.0)

        problem.solve()
        self.assertNumpyEqual(x.value, 6.0, msg="Hard constraint should be enforced")
        self.assertNumpyEqual(
            problem.slacks, 1.0, msg="Soft constraint should be slacking"
        )

    def test_polygon_constraint(self):
        problem = placo.Problem()

        # This is a unit square (drawn clockwise)
        polygon = np.array([[1.0, 1.0], [1.0, 2.0], [2.0, 2.0], [2.0, 1.0]])

        x = problem.add_variable(1)
        y = problem.add_variable(1)
        problem.add_constraint(
            placo.PolygonConstraint.in_polygon(x.expr(), y.expr(), polygon, 0.0)
        )
        problem.solve()

        self.assertNumpyEqual(
            np.hstack((x.value, y.value)),
            np.array(
                [
                    1.0,
                ]
            ),
            msg="The [0., 0.] value should be projected in the polygon bottom-left corner at [1., 1.]",
        )

        problem.add_constraint((x.expr() / y.expr()) == np.array([3.0, 3.0])).configure(
            "soft", 1.0
        )
        problem.solve()
        self.assertNumpyEqual(
            np.hstack((x.value, y.value)),
            np.array([2.0, 2.0]),
            msg="The [3., 3.] value should be projected in the polygon top-right corner at [2., 2.]",
        )

    def test_problem_overconstrained(self):
        """
        Checking that overconstrained equalities solve raise an exception
        """
        problem = placo.Problem()

        x = problem.add_variable(1)
        y = problem.add_variable(1)

        problem.add_constraint(x.expr() + y.expr() == 1.0)
        problem.add_constraint(x.expr() + y.expr() == 2.0)

        self.assertRaises(RuntimeError, problem.solve)

    def test_expr_t(self):
        """
        Testing expr_t, that allows adding constraints that are not aligned with integrator timesteps
        """
        problem = placo.Problem()

        xdd = problem.add_variable(10)
        integrator = placo.Integrator(xdd, np.array([0.0, 0.0]), 2, 0.1)

        # Adding constraint non aligned with timesteps
        problem.add_constraint(integrator.expr_t(0.5, 0) == 1.5)
        problem.add_constraint(integrator.expr_t(0.51, 0) == 2.5)

        for k in range(10):
            problem.add_limit(integrator.expr(k, 0), np.array([3.0]))

        problem.add_constraint(integrator.expr_t(1.0, 0) == 0.0)

        problem.solve()

        self.assertNumpyEqual(integrator.value(0.0, 0), 0.0)
        self.assertNumpyEqual(integrator.value(0.5, 0), 1.5)
        self.assertNumpyEqual(integrator.value(0.51, 0), 2.5)
        self.assertNumpyEqual(integrator.value(1.0, 0), 0.0)

    def test_integrator_zmp(self):
        """
        Testing using the integrator with the ZMP differential equation instead. Here:

        z = c - 1/(omega**2) ddc
        Thus: ddc = omega**2 c - omega**2 z

        If the state is, the control variable is delta zmps:
        [ c  ]
        [ dc ]
        [ z  ]
        [ dz ]
        """
        problem = placo.Problem()
        omega = 0.5

        dzmp = problem.add_variable(16)
        integrator = placo.Integrator(
            dzmp,
            np.array([0.0, 0.0, 0.0]),
            np.array(
                [
                    [0.0, 1.0, 0.0, 0.0],
                    [omega**2, 0.0, -(omega**2), 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                    [0.0, 0.0, 0.0, 0.0],
                ]
            ),
            0.1,
        )

        problem.add_constraint(integrator.expr(8, 0) == 0.5)
        problem.add_constraint(integrator.expr(16, 0) == 0.0)
        problem.add_constraint(integrator.expr(16, 1) == 0.0)
        problem.solve()

        self.assertNumpyEqual(integrator.value(0.8, 0), 0.5)
        self.assertNumpyEqual(integrator.value(1.6, 0), 0.0)
        self.assertNumpyEqual(integrator.value(1.6, 1), 0.0)

        # We compute ddc from the trajectory and checks that it matches the finite difference on velocity
        epsilon = 1e-8
        for t_test in [0.0, 0.25, 0.5, 1.5]:
            # Finite differences
            acc_fd = (
                integrator.value(t_test + epsilon, 1) - integrator.value(t_test, 1)
            ) / epsilon
            # ddc = omega**2 c - omega**2 z
            acc_zmp = omega**2 * integrator.value(
                t_test, 0
            ) - omega**2 * integrator.value(t_test, 2)

            self.assertNumpyEqual(acc_fd, acc_zmp, epsilon=1e-3)

    def test_active_constraints(self):
        """
        A simple test to check that the active constraints are tracked
        """
        problem = placo.Problem()
        x = problem.add_variable(1)
        y = problem.add_variable(1)

        problem.add_constraint((x.expr() + y.expr()) == 2)
        cst1 = problem.add_constraint(x.expr() >= 2.0)
        cst2 = problem.add_constraint(x.expr() >= 3.0)
        cst3 = problem.add_constraint(y.expr() >= -1.5)
        problem.solve()

        self.assertFalse(cst1.is_active)
        self.assertTrue(cst2.is_active)
        self.assertFalse(cst3.is_active)

    def test_active_constraints_with_bounds(self):
        """
        Active hard inequalities are reported when bounds and soft inequalities (slack variables) are also present
        """
        problem = placo.Problem()
        x = problem.add_variable(2)
        problem.add_bounds(x, 0, np.array([-10.0, -10.0]), np.array([10.0, 10.0]))
        problem.add_constraint(x.expr(1, 1) <= 5.0).configure("soft", 1.0)
        active = problem.add_constraint(x.expr(0, 1) <= 1.0)
        inactive = problem.add_constraint(x.expr(0, 1) >= -1.0)
        problem.add_constraint(x.expr() == np.array([3.0, 0.0])).configure("soft", 1.0)
        problem.solve()

        self.assertNumpyEqual(x.value, [1.0, 0.0])
        self.assertTrue(active.is_active)
        self.assertFalse(inactive.is_active)

    def test_badly_scaled_constraints(self):
        """
        Constraints with very small or large coefficients are enforced (the solver tolerances are absolute)
        """
        for scale in [1e-13, 1.0, 1e10]:
            problem = placo.Problem()
            x = problem.add_variable(2)
            problem.add_constraint(x.expr() == np.array([1.0, 1.0])).configure("soft", 1.0)
            problem.add_constraint(scale * x.expr(0, 1) <= scale * 0.5)
            problem.solve()
            self.assertNumpyEqual(x.value, [0.5, 1.0], msg=f"scale {scale}")

    def test_exactly_constrained(self):
        """
        Testing what happens if a problem is *exactly* constrained
        """
        problem = placo.Problem()
        x = problem.add_variable(1)
        y = problem.add_variable(1)

        problem.add_constraint((x.expr() + y.expr()) == 2.0)
        problem.add_constraint((x.expr() - y.expr()) == 1.0)

        problem.solve()

        self.assertNumpyEqual(x.value, 1.5)
        self.assertNumpyEqual(y.value, 0.5)

    def test_problem_polynom(self):
        problem = placo.Problem()

        coeffs = problem.add_variable(4)
        pp = placo.ProblemPolynom(coeffs)

        problem.add_constraint(pp.expr(0, 0) == 0)
        problem.add_constraint(pp.expr(0, 1) == 0)
        problem.add_constraint(pp.expr(1, 0) == 1)
        problem.add_constraint(pp.expr(1, 1) == 0)

        problem.solve()

        polynom = pp.get_polynom()
        self.assertEqual(len(polynom.coefficients), 4)
        self.assertNumpyEqual(polynom.value(0, 0), 0)
        self.assertNumpyEqual(polynom.value(0, 1), 0)
        self.assertNumpyEqual(polynom.value(1, 0), 1)
        self.assertNumpyEqual(polynom.value(1, 1), 0)

    def build_random_problem(self, rewrite_equalities, n, n_eq, n_ineq, seed):
        """
        Random feasible problem mixing hard equalities, soft equalities and hard inequalities
        """
        rng = np.random.default_rng(seed)
        problem = placo.Problem()
        problem.rewrite_equalities = rewrite_equalities
        x = problem.add_variable(n)
        x0 = rng.normal(size=n)

        A = rng.normal(size=(n_eq, n))
        problem.add_constraint(x.expr().left_multiply(A) == A @ x0)

        # Soft objective pulling away from the feasible point, so that some inequalities are active
        problem.add_constraint(x.expr() == 3 * rng.normal(size=n)).configure("soft", 1.0)

        G = rng.normal(size=(n_ineq, n))
        h = G @ x0 + 0.1
        problem.add_constraint(x.expr().left_multiply(G) <= h)

        problem.solve()
        return x.value, A, x0, G, h

    def test_equality_elimination(self):
        """
        Eliminating hard equalities (rewrite_equalities, QR decomposition) should give the same solution as
        keeping them in the QP, both with few equalities (many free variables) and many equalities (few free
        variables)
        """
        for n, n_eq, n_ineq in [(40, 4, 20), (30, 24, 10)]:
            for seed in range(3):
                x_rewrite, A, x0, G, h = self.build_random_problem(True, n, n_eq, n_ineq, seed)
                x_kkt, _, _, _, _ = self.build_random_problem(False, n, n_eq, n_ineq, seed)

                self.assertNumpyEqual(x_rewrite, x_kkt, msg=f"Solutions differ (n={n}, n_eq={n_eq}, seed={seed})")
                self.assertNumpyEqual(A @ x_rewrite, A @ x0, msg="Hard equalities should hold")
                self.assertTrue(np.all(G @ x_rewrite <= h + 1e-8), msg="Hard inequalities should hold")

    def test_soft_inequalities_analytical(self):
        """
        min ||x||^2 + ||x0 - 1 - s0||^2 + 3 ||x1 - 2 - s1||^2 with s >= 0: x0 = 0.5, x1 = 1.5 (soft
        inequalities x0 >= 1 and x1 >= 2). Also checked with a hard equality on a third variable, with and without
        rewriting the equalities.
        """
        for with_equality in [False, True]:
            for rewrite_equalities in [True, False]:
                problem = placo.Problem()
                problem.rewrite_equalities = rewrite_equalities
                x = problem.add_variable(3)
                problem.add_constraint(x.expr(0, 2) == 0.0).configure("soft", 1.0)
                problem.add_constraint(x.expr(0, 1) >= 1.0).configure("soft", 1.0)
                problem.add_constraint(x.expr(1, 1) >= 2.0).configure("soft", 3.0)
                if with_equality:
                    problem.add_constraint(x.expr(2, 1) == 5.0)
                else:
                    problem.add_constraint(x.expr(2, 1) == 5.0).configure("soft", 1.0)
                problem.solve()

                self.assertNumpyEqual(x.value, np.array([0.5, 1.5, 5.0]), epsilon=1e-5)

    def build_soft_inequalities_problem(self, rewrite_equalities, explicit_slacks, seed):
        """
        Random problem with hard/soft equalities and hard/soft inequalities. If explicit_slacks is True, soft
        inequalities are replaced with explicit slack variables (hard s >= 0 and soft G x + s = h)
        """
        rng = np.random.default_rng(seed)
        n, n_eq, n_ineq, n_soft = 20, 5, 10, 15
        problem = placo.Problem()
        problem.rewrite_equalities = rewrite_equalities
        x = problem.add_variable(n)
        x0 = rng.normal(size=n)

        A = rng.normal(size=(n_eq, n))
        problem.add_constraint(x.expr().left_multiply(A) == A @ x0)
        problem.add_constraint(x.expr() == 3 * rng.normal(size=n)).configure("soft", 1.0)
        G = rng.normal(size=(n_ineq, n))
        problem.add_constraint(x.expr().left_multiply(G) <= G @ x0 + 0.1)

        Gs = rng.normal(size=(n_soft, n))
        hs = Gs @ x0
        weight = 10.0
        if explicit_slacks:
            s = problem.add_variable(n_soft)
            problem.add_constraint(s.expr() >= 0.0)
            problem.add_constraint(x.expr().left_multiply(Gs) + s.expr() == hs).configure("soft", weight)
        else:
            problem.add_constraint(x.expr().left_multiply(Gs) <= hs).configure("soft", weight)

        problem.solve()
        return x.value

    def test_soft_inequalities_random(self):
        """
        Soft inequalities should give the same solution with or without rewriting the equalities, and the same
        solution as explicit slack variables
        """
        for seed in range(5):
            x_rewrite = self.build_soft_inequalities_problem(True, False, seed)
            x_no_rewrite = self.build_soft_inequalities_problem(False, False, seed)
            x_explicit = self.build_soft_inequalities_problem(True, True, seed)

            self.assertNumpyEqual(x_rewrite, x_no_rewrite, msg=f"rewrite_equalities changes the solution (seed {seed})")
            self.assertNumpyEqual(x_rewrite, x_explicit, msg=f"Explicit slacks give a different solution (seed {seed})")

    def test_bounds(self):
        """
        Bounds give the same solution as the equivalent hard inequalities, with or without the elimination of
        equalities. Bounds on the same values are merged.
        """
        inf = np.inf
        for rewrite in [True, False]:
            for seed in range(3):
                solutions = []
                for use_bounds in [False, True]:
                    rng = np.random.default_rng(seed)
                    problem = placo.Problem()
                    problem.rewrite_equalities = rewrite
                    x = problem.add_variable(4)
                    y = problem.add_variable(4)
                    for v in [x, y]:
                        problem.add_constraint(v.expr(0, 2).left_multiply(rng.normal(size=(1, 2))) == rng.normal())
                    # Pulling the solution out of the bounds
                    problem.add_constraint(x.expr() == 3.0 * rng.normal(size=4)).configure("soft", 1.0)
                    problem.add_constraint(y.expr() == 3.0 * rng.normal(size=4)).configure("soft", 1.0)

                    lower, upper = np.array([-1.0, -inf, -0.5, -2.0]), np.array([1.0, 0.5, inf, 2.0])
                    tighter_upper = np.array([0.8, 1.0, 1.0, 1.0])
                    if use_bounds:
                        problem.add_bounds(x, 0, lower, upper)
                        problem.add_bounds(x, 0, np.full(4, -inf), tighter_upper)
                        problem.add_bounds(y, 1, np.array([-0.3, -0.2]), np.array([0.3, 0.2]))
                    else:
                        for k in range(4):
                            if np.isfinite(lower[k]):
                                problem.add_constraint(x.expr(k, 1) >= lower[k])
                            problem.add_constraint(x.expr(k, 1) <= min(upper[k], tighter_upper[k]))
                        problem.add_constraint(y.expr(1, 2) >= np.array([-0.3, -0.2]))
                        problem.add_constraint(y.expr(1, 2) <= np.array([0.3, 0.2]))
                    problem.solve()
                    solutions.append(problem.x.copy())
                msg = f"rewrite={rewrite} seed={seed}"
                self.assertNumpyEqual(solutions[0], solutions[1], msg=msg)
                self.assertTrue(np.all(solutions[1][:4] <= np.minimum(upper, tighter_upper) + 1e-8), msg=msg)
                self.assertTrue(np.all(solutions[1][:4] >= lower - 1e-8), msg=msg)

        # Invalid sizes, and bounds being cleared with the constraints
        problem = placo.Problem()
        x = problem.add_variable(3)
        with self.assertRaises(RuntimeError):
            problem.add_bounds(x, 2, np.zeros(2), np.ones(2))
        with self.assertRaises(RuntimeError):
            problem.add_bounds(x, 0, np.zeros(2), np.ones(3))
        problem.add_bounds(x, 0, np.ones(3), np.ones(3))
        problem.add_constraint(x.expr() == 0.0).configure("soft", 1.0)
        problem.solve()
        self.assertNumpyEqual(x.value, np.ones(3))
        problem.clear_constraints()
        problem.add_constraint(x.expr() == 0.0).configure("soft", 1.0)
        problem.solve()
        self.assertNumpyEqual(x.value, np.zeros(3))

if __name__ == "__main__":
    unittest.main()
