import time
import placo
import argparse
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from footsteps_planner import draw_footsteps
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, footsteps_viz

parser = argparse.ArgumentParser(description="Process some integers.")
parser.add_argument("-g", "--graph", action="store_true", help="Plot")
parser.add_argument("-p", "--pybullet", action="store_true",
                    help="PyBullet visualization")
parser.add_argument("-m", "--meshcat", action="store_true",
                    help="MeshCat visualization")
args = parser.parse_args()

# Plotting
nb_plotted_steps = 10

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")
robot.load()
robot.ensure_on_floor()

# Creating the kinematics solver
solver = robot.make_solver()

# Creating the FootstepsPlanner
T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
T_world_right = placo.flatten_on_floor(robot.get_T_world_right())

####### Naive FootstepsPlanner ########
planner = placo.FootstepsPlannerNaive("left", T_world_left, T_world_right)
T_world_leftTarget = T_world_left.copy()
T_world_rightTarget = T_world_right.copy()
T_world_leftTarget[0, 3] += 1.0
T_world_rightTarget[0, 3] += 1.0
planner.configure(T_world_leftTarget, T_world_rightTarget)

##### Repetitive FootstepsPlanner #####
# planner = placo.FootstepsPlannerRepetitive("left", T_world_left, T_world_right)
# d_x = 0.1
# d_y = 0.
# d_theta = 0.
# nb_steps = 5
# planner.config(d_x, d_y, d_theta, nb_steps)

# Creating the pattern generator
walk = placo.WalkPatternGenerator(robot, solver, planner)
walk.parameters.dt = 0.025
walk.parameters.single_support_duration = .35
walk.parameters.double_support_duration = 0.0
walk.parameters.startend_double_support_duration = 0.5
walk.parameters.maximum_steps = 500
walk.parameters.walk_com_height = 0.32
walk.parameters.walk_foot_height = 0.04
walk.parameters.pendulum_height = 0.32
walk.parameters.walk_trunk_pitch = 0.2
walk.parameters.walk_foot_tilt = 0.2
walk.parameters.foot_length = 0.1576
walk.parameters.foot_width = 0.092
walk.parameters.feet_spacing = 0.122
walk.parameters.zmp_margin = 0.045

# Initial steps planification
planner.plan()
double_support = walk.parameters.double_support_duration / walk.parameters.dt > 1
planner.make_supports(True, double_support, True)

if args.graph:
    data_left = []
    data_right = []
    data = []
    draw_footsteps(walk.get_trajectory().footsteps, show=False)

if args.pybullet:
    import pybullet as p
    from onshape_to_robot.simulation import Simulation
    sim = Simulation("sigmaban/robot.urdf", realTime=True, dt=0.005)

if args.meshcat:
    viz = robot_viz(robot)
    footsteps_viz(walk.get_trajectory().footsteps)

start_t = time.time()
t = -2.
dt = 0.005

while nb_plotted_steps > 0:

    T = max(0, t)

    walk.next(dt)

    # Replan
    if str(previous_support) != robot.get_support_side() and str(previous_support) != "both":
        print("\nREPLAN")

        if args.graph:
            nb_plotted_steps -= 1

        # start_timer = time.time()

        # Naive
        T_world_leftTarget[0, 3] += 0.08
        T_world_rightTarget[0, 3] += 0.08
        planner.configure(T_world_leftTarget, T_world_rightTarget)

        # Repetitive
        # planner.configure(d_x, d_y, d_theta, nb_steps)

        planner.replan()

        # elapsed_time = time.time() - start_timer
        # print(f"Replan computation time: {elapsed_time*1e6}µs")

    previous_support = robot.get_support_side()

    if args.meshcat:
        viz.display(robot.state.q)
        robot_frame_viz(robot, "left_foot")
        robot_frame_viz(robot, "right_foot")
        com = robot.com_world()
        com[2] = 0
        point_viz("com", com)

    if args.pybullet:
        joints = {joint: robot.get_joint(joint)
                  for joint in sim.getJoints()}
        applied = sim.setJoints(joints)
        sim.tick()

    if args.graph:
        lf = walk.get_trajectory().get_T_world_left(T)
        data_left.append(lf[:3, 3])

        rf = walk.get_trajectory().get_T_world_right(T)
        data_right.append(rf[:3, 3])

        data.append(
            [walk.get_trajectory().com.pos(T), walk.get_trajectory().com.zmp(T), walk.get_trajectory().com.dcm(T)])

    # Spin-lock until the next tick
    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)

if args.graph:
    data_left = np.array(data_left)
    data_right = np.array(data_right)
    data = np.array(data)

    plt.plot(data.T[0][0], data.T[1][0], label="CoM", lw=3)
    plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
    plt.plot(data.T[0][2], data.T[1][2], label="DCM", lw=3)

    plt.legend()
    plt.grid()
    plt.show()
