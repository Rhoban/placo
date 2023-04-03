import unittest
import placo
import numpy as np

dt = 0.1
steps = 100


class TestJerkPlanner(unittest.TestCase):
    def setUp(self):
        """
        We instantiate a jerk planner starting at 0,0 with zero speed and acceleration
        We add equality constraint to reach 1,0 with zero speed and acceleration
        """
        self.jerk_planner = placo.JerkPlanner(steps, np.array([0.0, 0.0]), np.array([0.0, 0]), np.array([0.0, 0]), dt, 12.0)
        self.jerk_planner.add_equality_constraint(steps, np.array([1.0, 0.0]), placo.ConstraintType.position)
        self.jerk_planner.add_equality_constraint(steps, np.array([0.0, 0.0]), placo.ConstraintType.velocity)
        self.jerk_planner.add_equality_constraint(steps, np.array([0.0, 0.0]), placo.ConstraintType.acceleration)

    def test_jerk_planner(self):
        """
        Tests that the jerk planner enforce constraints
        """
        epsilon = 1e-6
        dt = 0.1

        self.jerk_planner.add_greater_than_constraint(50, np.array([1.0, 1.0]), placo.ConstraintType.position)

        trajectory = self.jerk_planner.plan()

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

    def test_values(self):
        """
        We give the planner some equality constraint and check if they are successfully enforced
        """
        values = [
            [10, np.array([1.0, 1.0]), placo.ConstraintType.position, lambda traj, t: traj.pos(t)],
            [20, np.array([-1.0, 2.0]), placo.ConstraintType.velocity, lambda traj, t: traj.vel(t)],
            [30, np.array([2.0, 3.0]), placo.ConstraintType.acceleration, lambda traj, t: traj.acc(t)],
            [40, np.array([-2.0, 4.0]), placo.ConstraintType.dcm, lambda traj, t: traj.dcm(t)],
            [50, np.array([1.0, -2.0]), placo.ConstraintType.zmp, lambda traj, t: traj.zmp(t)],
            [60, np.array([-2.0, 5.0]), placo.ConstraintType.dzmp, lambda traj, t: traj.dzmp(t)],
        ]

        for step, target, ctype, _ in values:
            self.jerk_planner.add_equality_constraint(step, target, ctype)

        trajectory = self.jerk_planner.plan()

        for step, target, _, getter in values:
            self.assertTrue(
                np.linalg.norm(getter(trajectory, step * dt) - target) < 1e-6,
                msg=f"Checking that we can successfully assign a constraint for {ctype}",
            )

    def test_soft_minimize(self):
        """
        Using soft equality constraint, we test that we can minimize other quantities than jerlk
        """

        jerk_trajectory = self.jerk_planner.plan()

        for step in range(steps):
            constraint = self.jerk_planner.add_equality_constraint(step, np.array([0.0, 0.0]), placo.ConstraintType.velocity)
            constraint.configure("soft", 1.0)

        vel_trajectory = self.jerk_planner.plan()

        costs_jerk = [0.0, 0.0]
        costs_vel = [0.0, 0.0]
        for k in range(steps):
            costs_jerk[0] += sum(jerk_trajectory.jerk(k * dt) ** 2)
            costs_jerk[1] += sum(vel_trajectory.jerk(k * dt) ** 2)

            costs_vel[0] += sum(jerk_trajectory.vel(k * dt) ** 2)
            costs_vel[1] += sum(vel_trajectory.vel(k * dt) ** 2)

        self.assertGreater(
            costs_jerk[1], costs_jerk[0], msg="The jerk cost of trajectory optimizing velocity should be higher"
        )
        self.assertGreater(
            costs_vel[0], costs_vel[1], msg="The velocity cost of a trajectory optimizing jerk should be higher"
        )


if __name__ == "__main__":
    unittest.main()
