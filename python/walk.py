import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
import tf
import matplotlib.pyplot as plt
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz
from footsteps_planner import draw_footsteps

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")
robot.ensure_on_floor()

T_world_left = robot.get_T_world_left()
T_world_right = robot.get_T_world_right()

T_world_leftTarget = T_world_left.copy()
T_world_rightTarget = T_world_right.copy()

T_world_leftTarget[0, 3] += 1.
T_world_rightTarget[0, 3] += 1.

walk = placo.WalkPatternGenerator(robot)
walk.dt = 0.05
walk.omega = np.sqrt(9.81/0.37)
trajectory = walk.plan(T_world_leftTarget, T_world_rightTarget)

ts = np.linspace(0, trajectory.com.duration(), 1000)
data = np.array([[trajectory.com.pos(t), trajectory.com.zmp(t), trajectory.com.dcm(t)] for t in ts])

draw_footsteps(trajectory.footsteps, show=False)
plt.plot(data.T[0][0], data.T[1][0], label="CoM", lw=3)
plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
plt.plot(data.T[0][2], data.T[1][2], label="DCM", lw=3)
plt.legend()
plt.show()
