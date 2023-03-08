import unittest
import placo
import numpy as np


class TestJerkPlanner(unittest.TestCase):
    def test_jerk_planner(self):
        epsilon = 1e-6
        dt = 0.1

        jerk_planner = placo.JerkPlanner(100, np.array([0.0, 0.0]), np.array([0.0, 0]), np.array([0.0, 0]), dt, 0.0)

        jerk_planner.add_equality_constraint(100, np.array([1.0, 0.0]), placo.ConstraintType.position)
        jerk_planner.add_equality_constraint(100, np.array([0.0, 0.0]), placo.ConstraintType.velocity)
        jerk_planner.add_equality_constraint(100, np.array([0.0, 0.0]), placo.ConstraintType.acceleration)

        jerk_planner.add_greater_than_constraint(50, np.array([1.0, 1.0]), placo.ConstraintType.position)

        trajectory = jerk_planner.plan()

        # Checking that we start on proper position/velocity
        self.assertAlmostEqual(
            np.linalg.norm(trajectory.pos(0)), 0.0, msg="Jerk planner initial position should be the one asked"
        )
        self.assertAlmostEqual(
            np.linalg.norm(trajectory.vel(0)), 0.0, msg="Jerk planner initial velocity should be the one asked"
        )
        self.assertAlmostEqual(
            np.linalg.norm(trajectory.acc(0)), 0.0, msg="Jerk planner initial acceleration should be the one asked"
        )

        # Checking that we reach the asked target
        self.assertAlmostEqual(
            np.linalg.norm(trajectory.pos(10.0) - np.array([1.0, 0.0])),
            0.0,
            msg="Jerk planner final position should be the one asked",
        )
        self.assertAlmostEqual(
            np.linalg.norm(trajectory.vel(10)), 0.0, msg="Jerk planner final velocity should be the one asked"
        )
        self.assertAlmostEqual(
            np.linalg.norm(trajectory.acc(10)), 0.0, msg="Jerk planner final acceleration should be the one asked"
        )

        # Checking the inequality at step 50
        self.assertGreaterEqual(trajectory.pos(5.0)[0] + epsilon, 1.0, msg="Jerk planner inequality should be enforced")
        self.assertGreaterEqual(trajectory.pos(5.0)[1] + epsilon, 1.0, msg="Jerk planner inequality should be enforced")


if __name__ == "__main__":
    unittest.main()
