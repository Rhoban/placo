import unittest
import placo
import numpy as np


class TestUtils(unittest.TestCase):
    def assertNumpyEqual(self, a, b, msg=None, epsilon=1e-6):
        if msg is None:
            msg = f"Checking that {a} equals {b}"
        self.assertTrue(np.linalg.norm(a - b) < epsilon, msg=msg)

    def test_cubic_spline(self):
        """
        Some basic tests on the Cubic spline
        """
        spline = placo.CubicSpline()
        spline.add_point(0.0, 1.0, -1.0)
        spline.add_point(1.0, 2.0, 2.0)
        spline.add_point(2.0, 1.0, 4.0)

        # Testing values
        self.assertNumpyEqual(spline.pos(0), 1.0)
        self.assertNumpyEqual(spline.pos(1), 2.0)
        self.assertNumpyEqual(spline.pos(2), 1.0)

        self.assertNumpyEqual(spline.vel(0), -1.0)
        self.assertNumpyEqual(spline.vel(1), 2.0)
        self.assertNumpyEqual(spline.vel(2), 4.0)

        # Testing that "vel" is indeed the derivative of pos using finite differences
        epsilon = 1e-8
        for t in np.linspace(0, 1, 100):
            vel_fd = (spline.pos(t + epsilon) - spline.pos(t)) / epsilon
            vel = spline.vel(t)
            self.assertNumpyEqual(vel, vel_fd)

    def test_angle_spline(self):
        """
        Tests angle wrapping for angular splines
        """
        # Angular cubic spline should handle angle wrap.
        # For example, moving from -3.1 rad to 3.1 rad should actually go to negative (to avoid discontinuities)
        spline = placo.CubicSpline(True)

        spline.add_point(1.0, -3.1, 0.0)
        spline.add_point(2.0, 3.1, 0.0)
        self.assertTrue(spline.pos(1.25) < 3.1)

        spline = placo.CubicSpline(True)

        spline.add_point(6.3, 4.58, 0.0)
        spline.add_point(6.6, -1.4, 0.0)
        self.assertTrue(spline.pos(6.4) > 4.58)


if __name__ == "__main__":
    unittest.main()
