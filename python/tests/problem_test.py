import unittest
import placo
import numpy as np


class TestProblem(unittest.TestCase):
    def test_expression_arithmetics(self):
        problem = placo.Problem()
        x = problem.add_variable("x", 16)

        e = x.expr()
        self.assertTrue(np.linalg.norm(e.A - np.eye(16)) < 1e-6)
        self.assertTrue(np.linalg.norm(e.b - np.zeros(16)) < 1e-6)

        # Checking sums and multiplications
        self.assertTrue(np.linalg.norm((e + e).A - 2 * np.eye(16)) < 1e-6)
        self.assertTrue(np.linalg.norm((e - e).A) < 1e-6)
        self.assertTrue(np.linalg.norm((e * 2).A - 2 * np.eye(16)) < 1e-6)
        self.assertTrue(np.linalg.norm((2 * e).A - 2 * np.eye(16)) < 1e-6)

        # Checking sum
        self.assertTrue(np.linalg.norm((e + np.ones(16)).b - np.ones(16)) < 1e-6)
        self.assertTrue(np.linalg.norm((e - np.ones(16)).b + np.ones(16)) < 1e-6)
        # This one doesn't work because of how the overload is done
        # self.assertTrue(np.linalg.norm((np.ones(16) + e).b - np.ones(16)) < 1e-6)

        # Checking multiplication
        self.assertTrue(np.linalg.norm((e.multiply(np.eye(16) * 2)).A - 2 * np.eye(16)) < 1e-6)

    def test_solve(self):
        problem = placo.Problem()

        x = problem.add_variable("x", 16)
        problem.add_equality(x.expr().sum(), np.array([1.0]))
        problem.solve()
        self.assertTrue((x.value == 1 / 16.0).all(), msg="16 values which sum equals 1 should be minimized to 1/16")


if __name__ == "__main__":
    unittest.main()
