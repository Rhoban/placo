import unittest
import placo
import numpy as np


class TestProblem(unittest.TestCase):
    def assertNumpyEqual(self, a, b, msg=None, epsilon=1e-6):
        self.assertTrue(np.linalg.norm(a - b) < epsilon, msg=msg)

    def test_expression_arithmetics(self):
        problem = placo.Problem()
        x = problem.add_variable("x", 16)

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

    def test_stacking(self):
        problem = placo.Problem()

        x = problem.add_variable("x", 8)
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
        x = problem.add_variable("x", 16)
        problem.add_equality(x.expr().sum(), np.array([1.0]))
        problem.solve()
        self.assertNumpyEqual(x.value, 1 / 16.0, msg="16 values which sum equals 1 should be minimized to 1/16")

        # We add an inequality so that the 0th value should be greater than 2
        problem.add_greater_than(x.expr(0, 1), np.array([2.0]))
        problem.add_lower_than(x.expr(0, 1), np.array([10.0]))
        problem.solve()
        self.assertGreaterEqual(x.value[0], 2.0, msg=f"The 8th value should be >= 2")
        self.assertNumpyEqual(x.value[1:], -1 / 15.0, msg=f"The remaining values should be -1/15.")

    def test_expression_constraint(self):
        problem = placo.Problem()
        p1 = problem.add_variable("p1", 2)
        p2 = problem.add_variable("p2", 2)

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
        x = problem.add_variable("x", 32)
        integrator = placo.Integrator(x, np.array([0.0, 0.0, 0.0]), 3, 0.1)

        expected_A = np.array([[1.0, 0.1, 0.005], [0.0, 1.0, 0.1], [0.0, 0.0, 1.0]])
        expected_B = np.array([1 / 6 * 0.1**3, 0.005, 0.1])

        self.assertNumpyEqual(integrator.M, expected, msg="Checking system matrix M or order 3")
        self.assertNumpyEqual(integrator.A, expected_A, msg="Checking system matrix A or order 3")
        self.assertNumpyEqual(integrator.B, expected_B, msg="Checking system matrix B or order 3")


if __name__ == "__main__":
    unittest.main()
