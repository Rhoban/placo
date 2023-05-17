import unittest
import placo
import numpy as np
import matplotlib.pyplot as plt
from tf import translation, rotation
import time


class TestLIPM(unittest.TestCase):
    def assertNumpyEqual(self, a, b, msg=None, epsilon=1e-6):
        if msg is None:
            msg = f"Checking that {a} equals {b}"
        self.assertTrue(np.linalg.norm(a - b) < epsilon, msg=msg)

    def test_polygon_contains(self):
        """
        Simple polygon contains test
        """        
        footstep = placo.Footstep(0.05, 0.1)
        footstep.frame = np.eye(4)

        self.assertTrue(placo.Footstep.polygon_contains(footstep.support_polygon(), np.array([0.0, 0.0])))
        self.assertTrue(placo.Footstep.polygon_contains(footstep.support_polygon(), np.array([0.05, 0.0])))
        self.assertTrue(placo.Footstep.polygon_contains(footstep.support_polygon(), np.array([0.0, 0.023])))
        self.assertFalse(placo.Footstep.polygon_contains(footstep.support_polygon(), np.array([0.11, 0.0])))
        self.assertFalse(placo.Footstep.polygon_contains(footstep.support_polygon(), np.array([0.0, 0.1])))

    def test_overlap(self):
        """
        Simple overlap tests
        """        
        footstep1 = placo.Footstep(0.05, 0.1)
        footstep1.frame = np.eye(4)

        footstep2 = placo.Footstep(0.05, 0.1)
        footstep2.frame = translation((0.01, 0.01, 0.))

        footstep3 = placo.Footstep(0.05, 0.1)
        footstep3.frame = translation((0.2, 0.2, 0.))

        self.assertTrue(footstep1.overlap(footstep2, 0.))
        self.assertFalse(footstep1.overlap(footstep3, 0.))
        self.assertTrue(footstep1.overlap(footstep3, 0.15))


if __name__ == "__main__":
    unittest.main()
