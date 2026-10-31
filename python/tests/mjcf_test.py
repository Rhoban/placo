import os
import tempfile
import unittest
import numpy as np
import placo

# Body with two joints, and the same robot where the second joint is in a massless child body
SEVERAL_JOINTS = """<mujoco model="several_joints">
  <compiler angle="radian"/>
  <worldbody>
    <body name="base">
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <body name="arm" pos="0 0.1 0" quat="0.9659 0 0.2588 0">
        <joint name="shoulder_x" axis="1 0 0" pos="0 0 0.02" range="-1 1"/>
        <joint name="shoulder_y" axis="0 1 0" pos="0.01 0 0" range="-1 1"/>
        <inertial pos="0 0 -0.1" mass="0.5" diaginertia="0.002 0.003 0.001"/>
        <site name="tip" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
</mujoco>"""

SPLIT_JOINTS = """<mujoco model="split_joints">
  <compiler angle="radian"/>
  <worldbody>
    <body name="base">
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <body name="shoulder" pos="0 0.1 0" quat="0.9659 0 0.2588 0">
        <joint name="shoulder_x" axis="1 0 0" pos="0 0 0.02" range="-1 1"/>
        <inertial pos="0 0 0" mass="0" diaginertia="0 0 0"/>
        <body name="arm">
          <joint name="shoulder_y" axis="0 1 0" pos="0.01 0 0" range="-1 1"/>
          <inertial pos="0 0 -0.1" mass="0.5" diaginertia="0.002 0.003 0.001"/>
          <site name="tip" pos="0 0 -0.2"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>"""


def load(contents: str) -> placo.RobotWrapper:
    with tempfile.TemporaryDirectory() as directory:
        filename = os.path.join(directory, "robot.xml")
        with open(filename, "w") as f:
            f.write(contents)
        return placo.RobotWrapper(filename, placo.Flags.mjcf)


class TestMJCF(unittest.TestCase):
    def test_several_joints_per_body(self):
        """
        The joints of a body are chained, each one keeping its name
        """
        robot = load(SEVERAL_JOINTS)
        split = load(SPLIT_JOINTS)

        self.assertEqual(list(robot.joint_names()), ["shoulder_x", "shoulder_y"])
        self.assertEqual(robot.model.nq, split.model.nq)
        self.assertTrue(np.allclose(robot.get_joint_limits("shoulder_y"), [-1, 1]))

        for robot_ in [robot, split]:
            robot_.set_joint("shoulder_x", 0.3)
            robot_.set_joint("shoulder_y", -0.5)
            robot_.state.qd = np.linspace(-1, 1, robot_.model.nv)
            robot_.update_kinematics()

        for frame in ["arm", "tip"]:
            self.assertTrue(np.allclose(robot.get_T_world_frame(frame), split.get_T_world_frame(frame)))
            self.assertTrue(np.allclose(robot.frame_jacobian(frame, "local"), split.frame_jacobian(frame, "local")))
        self.assertTrue(np.allclose(robot.mass_matrix(), split.mass_matrix()))
        self.assertTrue(np.allclose(robot.non_linear_effects(), split.non_linear_effects()))
        self.assertTrue(np.allclose(robot.com_world(), split.com_world()))


if __name__ == "__main__":
    unittest.main()
