import unittest
import numpy as np
import placo

# A continuous joint (two configuration coordinates, cos and sin, for one degree of freedom) between two revolute
# joints: the solvers should not mix the configuration and velocity indices (see issue #48)
URDF = """<robot name="r">
  <link name="base"/><link name="arm"/><link name="wheel"/><link name="arm2"/>
  <joint name="a_arm" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="10"/></joint>
  <joint name="b_wheel" type="{kind}"><parent link="base"/><child link="wheel"/><axis xyz="0 0 1"/>
    <limit lower="-1000" upper="1000" effort="1" velocity="10"/></joint>
  <joint name="c_arm2" type="revolute"><parent link="wheel"/><child link="arm2"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="10" velocity="10"/></joint>
</robot>"""


def make_robot(kind):
    robot = placo.RobotWrapper("", 0, URDF.format(kind=kind))
    robot.update_kinematics()
    return robot


class TestContinuousJoints(unittest.TestCase):
    def test_joints_order(self):
        """
        The continuous joint is between the two revolute joints in the configuration
        """
        robot = make_robot("continuous")
        self.assertEqual(robot.model.nq, robot.model.nv + 2)
        q = [robot.get_joint_offset(j) for j in ["a_arm", "b_wheel", "c_arm2"]]
        v = [robot.get_joint_v_offset(j) for j in ["a_arm", "b_wheel", "c_arm2"]]
        self.assertEqual(q, sorted(q))
        self.assertEqual(v, sorted(v))

    def test_kinematics_joint_limits(self):
        """
        Joint limits of the other joints are enforced with a continuous joint (issue #48)
        """
        for kind in ["revolute", "continuous"]:
            robot = make_robot(kind)
            solver = placo.KinematicsSolver(robot)
            solver.mask_fbase(True)
            solver.enable_joint_limits(True)
            task = solver.add_joints_task()
            task.set_joints({"a_arm": 1.5, "c_arm2": -3.0})
            for _ in range(300):
                solver.solve(True)
                robot.update_kinematics()
            self.assertAlmostEqual(robot.get_joint("a_arm"), 0.5, places=6, msg=kind)
            self.assertAlmostEqual(robot.get_joint("c_arm2"), -2.0, places=6, msg=kind)

    def test_dynamics_joint_limits(self):
        """
        Joint limits of the other joints are enforced by the dynamics solver with a continuous joint, the same way
        as with a revolute one
        """
        trajectories = {}
        for kind in ["revolute", "continuous"]:
            robot = make_robot(kind)
            solver = placo.DynamicsSolver(robot)
            solver.mask_fbase(True)
            solver.dt = 0.001
            solver.enable_joint_limits(True)
            task = solver.add_joints_task()
            task.set_joints({"a_arm": 1.5, "c_arm2": -3.0})
            trajectory = []
            for _ in range(3000):
                solver.solve(True)
                robot.update_kinematics()
                trajectory.append([robot.get_joint("a_arm"), robot.get_joint("c_arm2")])
            trajectories[kind] = np.array(trajectory)
            self.assertLess(trajectories[kind][:, 0].max(), 0.5 + 1e-2, msg=kind)
            self.assertGreater(trajectories[kind][:, 1].min(), -2.0 - 1e-2, msg=kind)
        self.assertTrue(np.allclose(trajectories["revolute"], trajectories["continuous"], atol=1e-6))

    def test_gear_tasks(self):
        """
        Gear tasks between joints after a continuous joint use their own positions
        """
        for solver_type in [placo.KinematicsSolver, placo.DynamicsSolver]:
            for kind in ["revolute", "continuous"]:
                robot = make_robot(kind)
                robot.set_joint("a_arm", 0.2)
                robot.set_joint("c_arm2", 0.1)
                robot.update_kinematics()
                solver = solver_type(robot)
                solver.mask_fbase(True)
                solver.dt = 0.001 if solver_type is placo.DynamicsSolver else 0.01
                gear = solver.add_gear_task()
                gear.set_gear("c_arm2", "a_arm", 2.0)
                gear.configure("gear", "hard", 1.0)
                for _ in range(2000 if solver_type is placo.DynamicsSolver else 50):
                    solver.solve(True)
                    robot.update_kinematics()
                # c_arm2 = 2 * b_arm
                self.assertAlmostEqual(
                    robot.get_joint("c_arm2"), 2 * robot.get_joint("a_arm"), places=3, msg=f"{solver_type} {kind}"
                )

    def test_joint_space_half_spaces_constraint(self):
        """
        The joint-space half-spaces constraint (A q <= b, A having a column per configuration coordinate) applies to
        the right joints after a continuous joint
        """
        for kind in ["revolute", "continuous"]:
            robot = make_robot(kind)
            solver = placo.KinematicsSolver(robot)
            solver.mask_fbase(True)
            task = solver.add_joints_task()
            task.set_joints({"a_arm": -0.4, "c_arm2": 1.0})
            # b_arm + c_arm2 <= 0.3
            A = np.zeros((1, robot.model.nq))
            A[0, robot.get_joint_offset("a_arm")] = 1.0
            A[0, robot.get_joint_offset("c_arm2")] = 1.0
            constraint = solver.add_joint_space_half_spaces_constraint(A, np.array([0.3]))
            constraint.configure("half_spaces", "hard", 1.0)
            for _ in range(100):
                solver.solve(True)
                robot.update_kinematics()
            # Projection of the targets on the constraint, a_arm being stopped by its lower limit (-0.5)
            self.assertAlmostEqual(robot.get_joint("a_arm"), -0.5, places=4, msg=kind)
            self.assertAlmostEqual(robot.get_joint("c_arm2"), 0.8, places=4, msg=kind)


if __name__ == "__main__":
    unittest.main()
