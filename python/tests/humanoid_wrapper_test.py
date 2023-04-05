import unittest
import placo
import numpy as np
from pathlib import Path
from tf import translation, rotation

this_dir = Path(globals().get("__file__", "./_")).absolute().parent


class TestHumanoidWrapper(unittest.TestCase):
    def setUp(self):
        self.robot = placo.HumanoidRobot(f"{this_dir}/sigmaban/robot.urdf", placo.Flags.collision_as_visual)

    def test_humanoid_wrapper(self):
        """
        Performs basic checks on the wrapper
        """
        # Testing frames presence
        expected_frames = ["left_foot", "right_foot", "trunk"]
        found_frames = list(self.robot.frame_names())
        for frame in expected_frames:
            self.assertTrue(frame in found_frames, msg=f"Frame {frame} should be in the sigmaban URDF")


if __name__ == "__main__":
    unittest.main()
