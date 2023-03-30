import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
import tf
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")

# Creating the viewer
viz = robot_viz(robot)
t = 0
dt = 0.01
start_t = time.time()
robot.update_kinematics()
v = 0

while True:
    viz.display(robot.state.q)

    if str(robot.get_support_side()) == "right":
        v += dt*.1
        over = v > .3
    else:
        v -= dt*.1
        over = v < -.3

    robot.set_joint("right_hip_pitch", v)
    robot.set_joint("left_hip_pitch", -v)

    print(f"v={v}, over={over}")
    if over:
        robot.update_support_side(
            str(placo.HumanoidRobot.other_side(robot.get_support_side())))
    robot.ensure_on_floor()

    # Show some frames
    robot_frame_viz(robot, "camera")
    robot_frame_viz(robot, "left_foot")
    robot_frame_viz(robot, "right_foot")
    robot_frame_viz(robot, "trunk")

    # Show the CoM
    point_viz("com", robot.com_world(), radius=0.025, color=0xAAAAAA)

    # Spin-lock until the next tick
    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
