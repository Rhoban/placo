import unittest
import placo
import numpy as np
import os

this_dir = os.path.dirname(os.path.realpath(__file__))


class TestKinematicsSolver(unittest.TestCase):
    def setUp(self):
        self.robot = placo.RobotWrapper(f"{this_dir}/quadruped/robot.urdf", placo.Flags.collision_as_visual)
        self.solver = self.robot.make_solver()

    def test_add_remove_task(self):
        self.assertEqual(self.solver.tasks_count(), 0, msg="There should be initially no task")

        regularization = self.solver.add_regularization_task(1e-6)
        self.assertEqual(self.solver.tasks_count(), 1, msg="There should be one task")

        self.solver.remove_task(regularization)
        self.assertEqual(self.solver.tasks_count(), 0, msg="There should be no more task")

        frame_task = self.solver.add_frame_task("trunk", np.eye(4))
        self.assertEqual(self.solver.tasks_count(), 2, msg="There should be two tasks")

        self.solver.remove_task(frame_task)
        self.assertEqual(self.solver.tasks_count(), 0, msg="There should be no more task")

    def test_kinetic_energy_regularization_task(self):
        """
        The kinetic energy regularization task can be configured and removed from Python
        """
        self.solver.dt = 0.01
        task = self.solver.add_kinetic_energy_regularization_task(1e-4)
        self.assertIsInstance(task, placo.Task)
        task.configure("kinetic_energy", "soft", 1e-3)
        self.solver.solve(False)
        self.assertEqual(self.solver.tasks_count(), 1)

        self.solver.remove_task(task)
        self.assertEqual(self.solver.tasks_count(), 0)


    def test_warm_start(self):
        """
        Along a trajectory reaching out of the joint limits, each step gives the same solution with and without the
        warm start (previous active set tried first), and the warm start is used
        """
        robot = placo.RobotWrapper(f"{this_dir}/sigmaban/robot.urdf", placo.Flags.collision_as_visual)
        solver = placo.KinematicsSolver(robot)
        solver.mask_fbase(True)
        solver.enable_joint_limits(True)
        T_world_foot = robot.get_T_world_frame("left_foot")
        task = solver.add_position_task("left_foot", T_world_foot[:3, 3])
        solver.add_regularization_task(1e-4)

        warm_started = 0
        for step in range(100):
            task.target_world = T_world_foot[:3, 3] + np.array([0.2, 0.1, 0.05]) * np.sin(step * 0.05)
            solver.problem.warm_start = True
            dq_warm = solver.solve(False)
            warm_started += solver.problem.warm_started
            solver.problem.warm_start = False
            dq = solver.solve(True)
            robot.update_kinematics()
            np.testing.assert_allclose(dq_warm, dq, atol=1e-9, err_msg=f"The warm start changes the solution (step {step})")

        self.assertGreater(warm_started, 0, msg="The warm start was never used")

    def test_compact_tasks(self):
        """
        Task matrices (compact internally) are the expected Jacobians, and solutions with hard tasks are the same
        with or without elimination of the equalities (QR or sparse elimination)
        """
        robot = self.robot
        rng = np.random.default_rng(2)
        for joint in robot.joint_names():
            robot.set_joint(joint, rng.uniform(-0.5, 0.5))
        robot.update_kinematics()

        solver = placo.KinematicsSolver(robot)
        position = solver.add_position_task("leg", np.array([0.1, 0.2, 0.0]))
        orientation = solver.add_orientation_task("leg_2", np.eye(3))
        relative = solver.add_relative_position_task("trunk", "tip", np.zeros(3))
        distance = solver.add_distance_task("leg", "leg_2", 0.1)
        for task in [position, orientation, relative, distance]:
            task.update()
        self.assertTrue(np.allclose(position.A, robot.frame_jacobian("leg", "local_world_aligned")[:3]))
        self.assertTrue(np.allclose(orientation.A, robot.frame_jacobian("leg_2", "world")[3:]))
        self.assertTrue(np.allclose(relative.A, robot.relative_position_jacobian("trunk", "tip")))
        self.assertEqual(distance.A.size, robot.model.nv)

        # Hard tasks, solved with the three elimination modes
        solutions = []
        for mode in ["qr", "none", "sparse"]:
            solver = placo.KinematicsSolver(robot)
            solver.problem.rewrite_equalities = mode != "none"
            solver.problem.sparse_elimination = mode == "sparse"
            solver.add_position_task("tip", robot.get_T_world_frame("tip")[:3, 3]).configure("tip", "hard", 1.0)
            solver.add_frame_task("body", robot.get_T_world_frame("body")).configure("body", "hard", 1.0, 1.0)
            solver.add_position_task("leg_2", np.array([0.1, 0.1, 0.1])).configure("leg_2", "soft", 1.0)
            solver.add_regularization_task(1e-4)
            solver.enable_joint_limits(True)
            solutions.append(solver.solve(False))
        for solution in solutions[1:]:
            self.assertTrue(np.allclose(solution, solutions[0], atol=1e-8))

if __name__ == "__main__":
    unittest.main()
