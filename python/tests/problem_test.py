import unittest
import placo
import numpy as np


class TestProblem(unittest.TestCase):
    def assertNumpyEqual(self, a, b, msg=None, epsilon=1e-6):
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
        self.assertNumpyEqual(e.multiply(np.eye(16) * 2).A, 2 * np.eye(16))

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
        problem = placo.Problem()

        x = problem.add_variable(8)
        e = x.expr(0, 1) << x.expr(2, 1) << x.expr(4, 1) << x.expr(6, 1)

        A = np.zeros((4, 8))
        A[0, 0] = 1
        A[1, 2] = 1
        A[2, 4] = 1
        A[3, 6] = 1
        self.assertTrue(np.linalg.norm(A - e.A) < 1e-6, msg="Expected matrix obtained by stacking")

        b = np.zeros(4)
        self.assertTrue(np.linalg.norm(b - e.b) < 1e-6, msg="Expected vector obtained by stacking")

    def test_simple_solve(self):
        problem = placo.Problem()

        # A problem where the sum of all the 16 variables should be equal to 1
        x = problem.add_variable(16)
        problem.add_constraint(x.expr().sum() == np.array([1.0]))
        problem.solve()
        self.assertNumpyEqual(x.value, 1 / 16.0, msg="16 values which sum equals 1 should be minimized to 1/16")

        # We add an inequality so that the 0th value should be greater than 2
        problem.add_constraint(x.expr(0, 1) >= np.array([2.0]))
        problem.add_constraint(x.expr(0, 1) <= np.array([10.0]))
        problem.solve()
        self.assertGreaterEqual(x.value[0], 2.0, msg=f"The 8th value should be >= 2")
        self.assertNumpyEqual(x.value[1:], -1 / 15.0, msg=f"The remaining values should be -1/15.")

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

        self.assertNumpyEqual(p1.value, np.array([17, 22]), msg="P1 should be in 17, 22")
        self.assertNumpyEqual(p2.value, np.array([18, 19]), msg="P2 should be in 18, 19")

    def test_integrator_matrix(self):
        M = placo.Integrator.continuous_system_matrix(3)

        expected = np.zeros((4, 4))
        expected[:3] = np.eye(4)[1:]

        self.assertTrue((M == expected).all(), msg="Checking system matrix or order 3")

        problem = placo.Problem()
        x = problem.add_variable(32)
        integrator = placo.Integrator(x, np.array([0.0, 0.0, 0.0]), 3, 0.1)

        expected_A = np.array([[1.0, 0.1, 0.005], [0.0, 1.0, 0.1], [0.0, 0.0, 1.0]])
        expected_B = np.array([1 / 6 * 0.1**3, 0.005, 0.1])

        self.assertNumpyEqual(integrator.M, expected, msg="Checking system matrix M or order 3")
        self.assertNumpyEqual(integrator.A, expected_A, msg="Checking system matrix A or order 3")
        self.assertNumpyEqual(integrator.B, expected_B, msg="Checking system matrix B or order 3")

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
        self.assertTrue(integrator.value(0.5, 0) < -5.0)

    def test_soft_inequality(self):
        problem = placo.Problem()
        x = problem.add_variable(1)

        inequality = problem.add_constraint(x.expr() >= 5.0)
        inequality.configure("soft", 1.0)

        inequality = problem.add_constraint(x.expr() >= 6.0)

        problem.solve()
        self.assertNumpyEqual(x.value, 6.0, msg="Hard constraint should be enforced")
        self.assertNumpyEqual(problem.slacks, 1.0, msg="Soft constraint should be slacking")

    def test_polygon_constraint(self):
        problem = placo.Problem()

        # This is a unit square (drawn clockwise)
        polygon = np.array([[1.0, 1.0], [1.0, 2.0], [2.0, 2.0], [2.0, 1.0]])

        xy = problem.add_variable(2)
        placo.PolygonConstraint.add_polygon_constraint(problem, xy.expr(), polygon, 0.0)
        problem.solve()
        self.assertNumpyEqual(
            xy.value,
            np.array([1.0, 1.0]),
            msg="The [0., 0.] value should be projected in the polygon bottom-left corner at [1., 1.]",
        )

        problem.add_constraint(xy.expr() == np.array([3.0, 3.0])).configure("soft", 1.0)
        problem.solve()
        self.assertNumpyEqual(
            xy.value,
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

        problem.add_constraint(x.expr() + y.expr() == 1.)
        problem.add_constraint(x.expr() + y.expr() == 2.)

        self.assertRaises(RuntimeError, problem.solve)


if __name__ == "__main__":
    unittest.main()
