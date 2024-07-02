import unittest
import placo
import numpy as np
import matplotlib.pyplot as plt
from placo_utils.tf import tf
import time


class TestTools(unittest.TestCase):
    def assertNumpyEqual(self, a, b, msg=None, epsilon=1e-6):
        if msg is None:
            msg = f"Checking that {a} equals {b}"
        self.assertTrue(np.linalg.norm(a - b) < epsilon, msg=msg)

    def test_wrap_angle(self):
        for k in range(-10, 10):
            self.assertNumpyEqual(placo.wrap_angle(k), np.arctan2(np.sin(k), np.cos(k)))

    def test_optimal_transformation(self):
        points_b = [
            [1.0, 2.0, 3.0],
            [-4.0, 5.0, 10.0],
            [-5.0, 0.0, 2.0],
            [-1.0, 9.0, 8.0],
        ]

        T_a_b = tf.translation_matrix([1.0, 2.0, 3.0]) @ tf.rotation_matrix(
            1.0, [0.0, 0.0, 1.0]
        )
        points_a = [(T_a_b @ np.array([x, y, z, 1.0]))[:3] for x, y, z in points_b]

        T_a_b_est = placo.optimal_transformation(np.array(points_a), np.array(points_b))

        self.assertNumpyEqual(T_a_b, T_a_b_est)


if __name__ == "__main__":
    unittest.main()
