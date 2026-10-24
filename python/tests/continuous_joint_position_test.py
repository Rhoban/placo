import unittest
import numpy as np
import placo

# A continuous joint has two configuration coordinates (cos and sin of its angle) for one degree of freedom (see
# issue #48)
URDF = """<robot name="r">
  <link name="base"/><link name="wheel"/><link name="arm"/>
  <joint name="wheel" type="continuous"><parent link="base"/><child link="wheel"/><axis xyz="0 0 1"/>
    <limit effort="1" velocity="10"/></joint>
  <joint name="arm" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="10"/></joint>
</robot>"""


def make_robot():
    robot = placo.RobotWrapper("", 0, URDF)
    robot.update_kinematics()
    return robot


def wheel_coordinates(robot):
    k = robot.get_joint_offset("wheel")
    return robot.state.q[k : k + 2]


class TestContinuousJointPosition(unittest.TestCase):
    def test_set_get_joint(self):
        """
        set_joint and get_joint use the angle of a continuous joint (its two coordinates being its cos and sin)
        """
        robot = make_robot()
        self.assertEqual(robot.get_joint_size("wheel"), 2)
        for angle in [0.0, 0.3, -1.2, 3.0, -3.0, np.pi / 2]:
            robot.set_joint("wheel", angle)
            self.assertAlmostEqual(robot.get_joint("wheel"), angle)
            self.assertTrue(np.allclose(wheel_coordinates(robot), [np.cos(angle), np.sin(angle)]))

        # Angles are given back in ]-pi, pi]
        robot.set_joint("wheel", 4.0)
        self.assertAlmostEqual(robot.get_joint("wheel"), 4.0 - 2 * np.pi)

        # Revolute joints are unchanged
        robot.set_joint("arm", 0.25)
        self.assertEqual(robot.get_joint("arm"), 0.25)
        self.assertEqual(robot.state.q[robot.get_joint_offset("arm")], 0.25)

    def test_joint_limits(self):
        """
        A continuous joint has no position limits
        """
        robot = make_robot()
        lower, upper = robot.get_joint_limits("wheel")
        self.assertEqual(lower, -np.inf)
        self.assertEqual(upper, np.inf)
        with self.assertRaises(RuntimeError):
            robot.set_joint_limits("wheel", -1.0, 1.0)

        robot.set_joint_limits("arm", -0.3, 0.4)
        self.assertTrue(np.allclose(robot.get_joint_limits("arm"), [-0.3, 0.4]))

    def test_q_noise(self):
        """
        The noise keeps the configuration of a continuous joint on the unit circle, and changes its angle
        """
        robot = make_robot()
        robot.set_joint("wheel", 0.5)
        robot.set_joint("arm", 0.1)
        angles = []
        for _ in range(50):
            robot.add_q_noise(0.1)
            self.assertAlmostEqual(np.linalg.norm(wheel_coordinates(robot)), 1.0)
            angles.append(robot.get_joint("wheel"))
            self.assertTrue(-0.5 <= robot.get_joint("arm") <= 0.5)
        self.assertGreater(np.ptp(angles), 1e-3)

    def test_joints_task(self):
        """
        A joints task drives a continuous joint to a target angle, with both solvers
        """
        for solver_type in [placo.KinematicsSolver, placo.DynamicsSolver]:
            robot = make_robot()
            solver = solver_type(robot)
            solver.mask_fbase(True)
            solver.dt = 0.001 if solver_type is placo.DynamicsSolver else 0.01
            task = solver.add_joints_task()
            task.set_joints({"wheel": 2.5})
            for _ in range(5000 if solver_type is placo.DynamicsSolver else 100):
                solver.solve(True)
                robot.update_kinematics()
            self.assertAlmostEqual(robot.get_joint("wheel"), 2.5, places=3, msg=str(solver_type))
            self.assertAlmostEqual(np.linalg.norm(wheel_coordinates(robot)), 1.0, msg=str(solver_type))


if __name__ == "__main__":
    unittest.main()
