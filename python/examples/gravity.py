import time
import placo
import pinocchio as pin
import numpy as np

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")

robot.reset()
robot.update_kinematics()

torques = robot.static_gravity_compensation_torques_dict("left_foot")
for dof in torques:
    print(f"{dof}: {torques[dof]}")
