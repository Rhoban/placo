import unittest
import placo
import numpy as np


class TestSwingFoot(unittest.TestCase):
    def test_swing(self):
        epsilon = 1e-6

        for t_start, t_end, height, initial, target in [
            [0.0, 2.0, 0.5, np.array([1.0, 1.0, 0.0]), np.array([2.0, 2.0, 0.0])],
            [2.0, 4.0, 0.25, np.array([-1.0, 1.0, 0.0]), np.array([2.0, 0.0, 0.0])],
            [10.0, 10.5, 0.1, np.array([0.0, -1.0, 0.0]), np.array([0.0, 2.0, 0.0])],
            [0.5, 1.25, 0.05, np.array([0.0, 1.0, 0.0]), np.array([2.0, 0.0, 0.0])],
        ]:
            trajectory = placo.SwingFoot.make_trajectory(t_start, t_end, height, initial, target)

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
                    trajectory.pos(t_start + (t_end - t_start) * ratio)[2] + epsilon,
                    height,
                    msg=f"The height at {ratio*100} of the trajectory should be greater than the one asked",
                )

    def test_swing_remake_same(self):
        """
        Planning a swing trajectory and remaking it with the same target should result in the same trajectory
        """
        trajectory = placo.SwingFoot.make_trajectory(0.0, 2.0, 0.25, np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.25, 0.0]))

        new_trajectory = placo.SwingFoot.remake_trajectory(trajectory, 1.28, np.array([1.0, 0.25, 0.0]))

        for t in np.linspace(0.0, 2.0, 32):
            self.assertAlmostEqual(
                np.linalg.norm(trajectory.pos(t) - new_trajectory.pos(t)),
                0.0,
                msg="The new trajectory should match the old one",
            )
            self.assertAlmostEqual(
                np.linalg.norm(trajectory.vel(t) - new_trajectory.vel(t)),
                0.0,
                msg="The new trajectory should match the old one",
            )

    def test_swing_remake(self):
        """
        Planning a footstep trajectory and remaking it with a different target
        """
        trajectory = placo.SwingFoot.make_trajectory(0.0, 2.0, 0.25, np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.25, 0.0]))

        new_target = np.array([0.8, 0.1, 0.0])
        new_trajectory = placo.SwingFoot.remake_trajectory(trajectory, 0.5, new_target)

        pos_before = trajectory.pos(0.5)
        pos_new = new_trajectory.pos(0.5)
        self.assertAlmostEqual(
            np.linalg.norm(pos_before - pos_new), 0.0, msg="The new trajectory should match the old one at the replan time"
        )

        vel_before = trajectory.pos(0.5)
        vel_new = new_trajectory.pos(0.5)
        self.assertAlmostEqual(
            np.linalg.norm(vel_before - vel_new),
            0.0,
            msg="The new trajectory vel. should match the old one at the replan time",
        )

        pos_new = new_trajectory.pos(2.0)
        self.assertAlmostEqual(np.linalg.norm(pos_new - new_target), 0.0, msg="The new trajectory should reach the new target")

        vel_new = new_trajectory.vel(2.0)[:2]
        self.assertAlmostEqual(np.linalg.norm(vel_new), 0.0, msg="The new trajectory should end with zero velocity")


if __name__ == "__main__":
    unittest.main()
