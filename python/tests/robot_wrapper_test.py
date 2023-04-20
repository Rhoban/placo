import unittest
import placo
import numpy as np
from pathlib import Path
from tf import translation, rotation

this_dir = Path(globals().get("__file__", "./_")).absolute().parent


class TestWrapper(unittest.TestCase):
    def setUp(self):
        self.robot = placo.RobotWrapper(f"{this_dir}/quadruped/robot.urdf", placo.Flags.collision_as_visual)

    def test_wrapper(self):
        """
        Performs basic checks on the wrapper
        """
        # Testing frames presence
        expected_frames = ["body", "trunk", "tip"]
        found_frames = list(self.robot.frame_names())
        for frame in expected_frames:
            self.assertTrue(frame in found_frames, msg=f"Frame {frame} should be in the quadruped URDF")

        # Checking DoF size (with floating base)
        self.assertEqual(12 + 7, len(self.robot.state.q))
        self.assertEqual(12 + 6, len(self.robot.state.qd))
        self.robot.update_kinematics()

        T_world_body = self.robot.get_T_world_frame("body")
        T_world_tip = self.robot.get_T_world_frame("tip")
        body_world = T_world_body[:3, 3]
        leg_world = T_world_tip[:3, 3]

        # Checking frames orientations
        self.assertAlmostEqual(np.linalg.norm(T_world_body - np.eye(4)), 0.0, msg="Body frame should be identity")
        self.assertAlmostEqual(np.linalg.norm(T_world_tip[:3, :3] - np.eye(3)), 0.0, msg="Leg tip frame is identity")

        # Checking a known distance
        self.assertAlmostEqual(np.linalg.norm(leg_world - body_world), 0.253, 2, msg="Distance from body to leg tip")

        # Testing T_a_b consistency
        T_body_tip = np.linalg.inv(T_world_body) @ T_world_tip
        self.assertAlmostEqual(
            np.linalg.norm(T_body_tip - self.robot.get_T_a_b("body", "tip")), 0.0, msg="Body frame should be identity"
        )

    def test_wrapper_content(self):
        """
        Testing loading a robot from an URDF stream instead of URDF file
        """
        urdf = """
        <robot name="test">
            <link name="base">
            </link>
        </robot>
        """
        robot = placo.RobotWrapper(f"", placo.Flags.collision_as_visual, urdf)

        self.assertEqual(7, len(robot.state.q))

    def test_change_dof(self):
        """
        Moving one DoF and checking that the tip leg indeed moves
        """
        self.assertAlmostEqual(self.robot.get_joint("leg3_a"), 0.0, msg="leg3_a should initially be 0")

        T_world_tip1 = self.robot.get_T_world_frame("tip")[:3, 3]
        self.robot.set_joint("leg3_a", 1.0)
        self.robot.update_kinematics()
        T_world_tip2 = self.robot.get_T_world_frame("tip")[:3, 3]

        self.assertAlmostEqual(self.robot.get_joint("leg3_a"), 1.0, msg="leg3_a should be 1")

        self.assertTrue(
            np.linalg.norm(T_world_tip1 - T_world_tip2) > 0.05, msg="Moving leg3a via set_joint should move the leg tip"
        )

    def test_change_state(self):
        """
        We also move one DoF but by accessing state().q
        """
        T_world_tip1 = self.robot.get_T_world_frame("tip")[:3, 3]

        offset = self.robot.get_joint_offset("leg3_a")
        q = self.robot.state.q
        q[offset] = 1
        self.robot.state.q = q
        self.robot.update_kinematics()

        T_world_tip2 = self.robot.get_T_world_frame("tip")[:3, 3]

        self.assertTrue(
            np.linalg.norm(T_world_tip1 - T_world_tip2) > 0.05, msg="Moving leg3a via state().q should move the leg tip"
        )

    def test_set_fbase(self):
        """
        We set the floating base to one target frame and check that the frame is indeed where we wanted it to be
        """
        T_world_body = translation((0.1, 0.2, 0.3)) @ rotation([0.0, 0.0, 1.0], 1.0)

        self.assertNotAlmostEqual(
            np.linalg.norm(self.robot.get_T_world_frame("body") - T_world_body), 0.0, msg="Body frame should be identity"
        )
        self.robot.set_T_world_frame("body", T_world_body)

        self.robot.update_kinematics()
        self.assertAlmostEqual(
            np.linalg.norm(self.robot.get_T_world_frame("body") - T_world_body), 0.0, msg="Body frame should be identity"
        )


if __name__ == "__main__":
    unittest.main()
