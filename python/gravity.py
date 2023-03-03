import time
import placo
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, footsteps_viz
import pinocchio as pin
import numpy as np

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")

robot.reset()
robot.update_kinematics()

torques = robot.static_gravity_compensation_torques_dict("left_foot")
for dof in torques:
    print(f"{dof}: {torques[dof]}")
