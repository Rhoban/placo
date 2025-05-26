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


if __name__ == "__main__":
    unittest.main()
