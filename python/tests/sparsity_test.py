import unittest
import placo
import numpy as np


class TestSparsity(unittest.TestCase):
    def assertNumpyEqual(self, a, b, msg=None, epsilon=1e-6):
        if msg is None:
            msg = f"Checking that {a} equals {b}"
        self.assertTrue(np.linalg.norm(a - b) < epsilon, msg=msg)

    def test_sparsity(self):
        sparsity = placo.Sparsity()
        sparsity.add_interval(1, 3)

        self.assertNumpyEqual(sparsity.intervals, [[1, 3]])

        sparsity.add_interval(5, 8)
        self.assertNumpyEqual(sparsity.intervals, [[1, 3], [5, 8]])

        sparsity.add_interval(7, 12)
        self.assertNumpyEqual(sparsity.intervals, [[1, 3], [5, 12]])

        sparsity.add_interval(0, 2)
        self.assertNumpyEqual(sparsity.intervals, [[0, 3], [5, 12]])

        sparsity.add_interval(14, 16)
        self.assertNumpyEqual(sparsity.intervals, [[0, 3], [5, 12], [14, 16]])

        sparsity.add_interval(2, 15)
        self.assertNumpyEqual(sparsity.intervals, [[0, 16]])

    def test_sparsity2(self):
        sparsity = placo.Sparsity()
        sparsity.add_interval(1, 2)

        self.assertNumpyEqual(sparsity.intervals, [[1, 2]])

        sparsity.add_interval(7, 9)
        self.assertNumpyEqual(sparsity.intervals, [[1, 2], [7, 9]])

        sparsity.add_interval(4, 5)
        self.assertNumpyEqual(sparsity.intervals, [[1, 2], [4, 5], [7, 9]])

        sparsity.add_interval(2, 4)
        self.assertNumpyEqual(sparsity.intervals, [[1, 5], [7, 9]])

        sparsity.add_interval(0, 10)
        self.assertNumpyEqual(sparsity.intervals, [[0, 10]])

    def test_sparsity_sum(self):
        sparsity1 = placo.Sparsity()
        sparsity1.add_interval(1, 2)

        sparsity2 = placo.Sparsity()
        sparsity2.add_interval(5, 6)

        sparsity = sparsity1 + sparsity2
        self.assertNumpyEqual(sparsity.intervals, [[1, 2], [5, 6]])

    def test_use_sparsity_same(self):
        """
        Checks that we obtain the same solution for a given problem with sparsity enabled or disabled
        """
        N = 64
        problem = placo.Problem()

        xdd = problem.add_variable(N)
        ydd = problem.add_variable(N)
        integrator_x = placo.Integrator(xdd, np.array([0.0, 0.0, 0.0]), 3, 1 / N)
        integrator_y = placo.Integrator(ydd, np.array([0.0, 0.0, 0.0]), 3, 1 / N)

        for k in range(N):
            problem.add_limit(integrator_x.expr(k, 1), np.array([7.5]))
            problem.add_limit(integrator_y.expr(k, 1), np.array([10.0]))

        for k in range(N):
            problem.add_constraint(integrator_x.expr(k, 0) == 1.5).configure("soft", 1.0)
            problem.add_constraint(integrator_y.expr(k, 0) == 1.5).configure("soft", 1.0)

        problem.add_constraint(integrator_x.expr(N, 0) == 1.0)
        problem.add_constraint(integrator_x.expr(N, 1) == 0.0)
        problem.add_constraint(integrator_x.expr(N, 2) == 0.0)
        problem.add_constraint(integrator_y.expr(N, 0) == 1.0)
        problem.add_constraint(integrator_y.expr(N, 1) == 0.0)
        problem.add_constraint(integrator_y.expr(N, 2) == 0.0)

        problem.add_constraint(integrator_x.expr(N // 2, 0) >= 2.0)
        problem.add_constraint(integrator_y.expr(N // 2, 0) <= -2.0)

        problem.add_constraint(integrator_x.expr(N // 4, 0) == integrator_y.expr(N // 4, 0))
        problem.add_constraint(integrator_x.expr(N // 4, 1) == integrator_y.expr(N // 4, 1))

        problem.use_sparsity = True
        problem.solve()
        solution1 = xdd.value.copy()

        problem.use_sparsity = False
        problem.solve()
        solution2 = xdd.value.copy()

        self.assertNumpyEqual(solution1, solution2)

    def test_detect_sparsity(self):
        M = np.array(
            [
                # 0   1    2    3    4    5    6    7
                [0.0, 1.0, 2.0, 0.0, 0.0, 1.0, 2.0, 3.0],
                [0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 2.0, 0.0],
            ]
        )

        sparsity = placo.Sparsity.detect_columns_sparsity(M)
        self.assertNumpyEqual(sparsity.intervals, [[1, 3], [5, 7]])


if __name__ == "__main__":
    unittest.main()
