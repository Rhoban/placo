import unittest
import placo
import numpy as np
from pathlib import Path

this_dir = Path(globals().get("__file__", "./_")).absolute().parent


class TestSwingFoot(unittest.TestCase):
    def test_wrapper(self):
        # Robot should load
        robot = placo.RobotWrapper(f"{this_dir}/quadruped/robot.urdf")

        # Testing frames presence
        expected_frames = ["body", "trunk", "tip"]
        found_frames = list(robot.frame_names())
        for frame in expected_frames:
            self.assertTrue(frame in found_frames, msg=f"Frame {frame} should be in the quadruped URDF")

        # Checking DoF size (with floating base)
        self.assertEqual(12 + 7, len(robot.state.q))
        self.assertEqual(12 + 6, len(robot.state.qd))
        robot.update_kinematics()

        # Checking a known distance
        T_world_body = robot.get_T_world_frame("body")
        T_world_tip = robot.get_T_world_frame("tip")
        body_world = T_world_body[:3, 3]
        leg_world = T_world_tip[:3, 3]

        self.assertAlmostEqual(np.linalg.norm(T_world_body - np.eye(4)), 0.0, msg="Body frame should be identity")
        self.assertAlmostEqual(np.linalg.norm(T_world_tip[:3, :3] - np.eye(3)), 0.0, msg="Leg tip frame is identity")
        self.assertAlmostEqual(np.linalg.norm(leg_world - body_world), 0.253, 2, msg="Distance from body to leg tip")


if __name__ == "__main__":
    unittest.main()
