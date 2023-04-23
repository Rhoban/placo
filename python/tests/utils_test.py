import unittest
import placo
import numpy as np


class TestUtils(unittest.TestCase):

    def test_angle_spline(self):
        spline = placo.PolySpline()

        spline.addPoint(6.3, 4.58, 0., True)
        spline.addPoint(6.6, -1.4, 0., True)
        spline.addPoint(6.8, -1.2, 0., True)

        ts = np.linspace(6.3, 6.8, 100)
        vs = [spline.get(t) for t in ts]
        import matplotlib.pyplot as plt
        plt.plot(ts, vs)
        plt.grid()
        plt.show()


if __name__ == "__main__":
    unittest.main()
