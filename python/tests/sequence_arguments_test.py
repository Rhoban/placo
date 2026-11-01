import os
import unittest
import placo
import numpy as np

this_dir = os.path.dirname(os.path.realpath(__file__))


class TestSequenceArguments(unittest.TestCase):
    """
    Lists and tuples can be passed to the bindings wherever a numpy array is expected
    """

    def setUp(self):
        self.robot = placo.RobotWrapper(f"{this_dir}/quadruped/robot.urdf", placo.Flags.collision_as_visual)
        self.robot.update_kinematics()
        self.solver = placo.KinematicsSolver(self.robot)

    def test_vectors(self):
        task = self.solver.add_com_task([0.0, 0.0, 0.1])
        self.assertTrue(np.allclose(task.target_world, [0.0, 0.0, 0.1]))

        task.target_world = (1, 2, 3)
        self.assertTrue(np.allclose(task.target_world, [1.0, 2.0, 3.0]))

        q = self.robot.state.q.copy()
        self.robot.state.q = list(q)
        self.assertTrue(np.allclose(self.robot.state.q, q))

    def test_matrices(self):
        T = self.robot.get_T_world_frame("trunk")
        T[:3, 3] = [0.1, 0.2, 0.3]

        self.robot.set_T_world_frame("trunk", T.tolist())
        self.robot.update_kinematics()
        self.assertTrue(np.allclose(self.robot.get_T_world_frame("trunk"), T))

        self.robot.set_T_world_frame("trunk", tuple(map(tuple, np.eye(4))))
        self.robot.update_kinematics()
        self.assertTrue(np.allclose(self.robot.get_T_world_frame("trunk"), np.eye(4)))

    def test_bounds(self):
        problem = placo.Problem()
        x = problem.add_variable(2)
        problem.add_bounds(x, 0, [0.5, 0.0], (1.0, 1.0))
        problem.solve()
        self.assertTrue(np.allclose(x.value, [0.5, 0.0]))

    def test_invalid(self):
        task = self.solver.add_com_task(np.zeros(3))
        for value in [[1.0, 2.0], ["a", "b", "c"], [[1.0, 2.0, 3.0]], "abc", range(3)]:
            with self.assertRaises(Exception):
                task.target_world = value


if __name__ == "__main__":
    unittest.main()
