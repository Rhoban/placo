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


if __name__ == "__main__":
    unittest.main()
