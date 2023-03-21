import unittest
import placo
import numpy as np


class TestSwingFoot(unittest.TestCase):
    def test_swing(self):
        epsilon = 1e-6

        for t_start, t_end, height, initial, target in [
            [0.0, 2.0, 0.5, np.array([1.0, 1.0, 0.0]),
             np.array([2.0, 2.0, 0.0])],
            [2.0, 4.0, 0.25, np.array([-1.0, 1.0, 0.0]),
             np.array([2.0, 0.0, 0.0])],
            [10.0, 10.5, 0.1, np.array(
                [0.0, -1.0, 0.0]), np.array([0.0, 2.0, 0.0])],
            [0.5, 1.25, 0.05, np.array(
                [0.0, 1.0, 0.0]), np.array([2.0, 0.0, 0.0])],
        ]:
            trajectory = placo.SwingFoot.make_trajectory(
                t_start, t_end, height, initial, target)

            self.assertAlmostEqual(
                np.linalg.norm(trajectory.pos(t_start) - initial),
                0,
                msg="The initial position of the swing foot should match the one asked",
            )
            self.assertAlmostEqual(
                np.linalg.norm(trajectory.pos(t_end) - target),
                0.0,
                msg="The final position of the swing foot should match the one asked",
            )
            self.assertAlmostEqual(
                np.linalg.norm(trajectory.vel(t_start)[:2]),
                0.0,
                msg="The initial velocity should not have a component on the gound plane",
            )
            self.assertAlmostEqual(
                np.linalg.norm(trajectory.vel(t_end)[:2]),
                0.0,
                msg="The final velocity should not have a component on the gound plane",
            )

            for ratio in [1 / 4, 2 / 4, 3 / 4]:
                self.assertGreaterEqual(
                    trajectory.pos(t_start + (t_end - t_start)
                                   * ratio)[2] + epsilon,
                    height,
                    msg=f"The height at {ratio*100} of the trajectory should be greater than the one asked",
                )


if __name__ == "__main__":
    unittest.main()
