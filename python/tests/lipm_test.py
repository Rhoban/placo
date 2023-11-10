import unittest
import placo
import numpy as np
import matplotlib.pyplot as plt
import time


class TestLIPM(unittest.TestCase):
    def assertNumpyEqual(self, a, b, msg=None, epsilon=1e-6):
        if msg is None:
            msg = f"Checking that {a} equals {b}"
        self.assertTrue(np.linalg.norm(a - b) < epsilon, msg=msg)

    def test_lipm(self):
        problem = placo.Problem()

        lipm = placo.LIPM(problem, 64, 0.1, np.array([0.0, 0.0]), np.array([0.0, 0.0]), np.array([0.0, 0.0]))

        problem.add_constraint(lipm.pos(64) == np.array([1.0, -1.0]))
        problem.add_constraint(lipm.vel(64) == np.array([0.0, 0.0]))
        problem.add_constraint(lipm.acc(64) == np.array([0.0, 0.0]))

        # for k in range(1, 65):
        #     problem.add_constraint(lipm.pos(k) == np.array([1.5, 1.5])).configure("soft", 1)

        polygon = np.array([[1.0, 1.0], [1.0, 2.0], [2.0, 2.0], [2.0, 1.0]])

        for k in range(32, 33):
            problem.add_constraint(placo.PolygonConstraint.in_polygon_xy(lipm.pos(k), polygon, 0.0))

        problem.solve()
        trajectory = lipm.get_trajectory()

        self.assertNumpyEqual(trajectory.pos(0.0), np.array([0.0, 0.0]))
        self.assertNumpyEqual(trajectory.vel(0.0), np.array([0.0, 0.0]))
        self.assertNumpyEqual(trajectory.acc(0.0), np.array([0.0, 0.0]))

        self.assertNumpyEqual(trajectory.pos(6.4), np.array([1.0, -1.0]))
        self.assertNumpyEqual(trajectory.vel(6.4), np.array([0.0, 0.0]))
        self.assertNumpyEqual(trajectory.acc(6.4), np.array([0.0, 0.0]))


if __name__ == "__main__":
    unittest.main()
