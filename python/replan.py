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

# Walk parameters
parameters = placo.HumanoidParameters()
parameters.dt = 0.025
parameters.single_support_duration = .35
parameters.double_support_duration = 0.0
parameters.startend_double_support_duration = 0.5
parameters.maximum_steps = 500
parameters.walk_com_height = 0.32
parameters.walk_foot_height = 0.04
parameters.pendulum_height = 0.32
parameters.walk_trunk_pitch = 0.2
parameters.walk_foot_tilt = 0.2
parameters.foot_length = 0.1576
parameters.foot_width = 0.092
parameters.feet_spacing = 0.122
parameters.zmp_margin = 0.045

# Creating the kinematics solver
solver = robot.make_solver()
task_holder = placo.SolverTaskHolder(robot, solver)

# Creating the FootstepsPlanner
T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
T_world_right = placo.flatten_on_floor(robot.get_T_world_right())

####### Naive FootstepsPlanner ########
# planner = placo.FootstepsPlannerNaive(parameters)
# T_world_leftTarget = T_world_left.copy()
# T_world_rightTarget = T_world_right.copy()
# T_world_leftTarget[0, 3] += 1.0
# T_world_rightTarget[0, 3] += 1.0
# planner.configure(T_world_leftTarget, T_world_rightTarget)

##### Repetitive FootstepsPlanner #####
planner = placo.FootstepsPlannerRepetitive(parameters)
d_x = 0.1
d_y = 0.
d_theta = 0.
nb_steps = 5
planner.configure(d_x, d_y, d_theta, nb_steps)

# Creating the pattern generator
walk = placo.WalkPatternGenerator(robot, planner, parameters)
trajectory = walk.plan()

if args.graph:
    data_left = []
    data_right = []
    data = []
    draw_footsteps(trajectory.supports, show=False)

if args.pybullet:
    import pybullet as p
    from onshape_to_robot.simulation import Simulation
    sim = Simulation("sigmaban/robot.urdf", realTime=True, dt=0.005)

if args.meshcat:
    viz = robot_viz(robot)
    footsteps_viz(trajectory.supports)

start_t = time.time()
t = -2.
dt = 0.005

while nb_plotted_steps > 0:
    T = max(0, t)

    task_holder.update_tasks(trajectory.get_T_world_left(T), trajectory.get_T_world_right(T),
                             trajectory.get_CoM_world(T), trajectory.get_R_world_trunk(T), False)

    previous_support = robot.get_support_side()

    robot.update_kinematics()
    robot.update_support_side(str(trajectory.support_side(T)))

    # Replan
    if str(previous_support) != robot.get_support_side() and str(previous_support) != "both":
        print("REPLAN")

        if args.graph:
            nb_plotted_steps -= 1

        # start_timer = time.time()

        # Naive
        # T_world_leftTarget[0, 3] += 0.08
        # T_world_rightTarget[0, 3] += 0.08
        # planner.configure(T_world_leftTarget, T_world_rightTarget)

        # Repetitive
        planner.configure(d_x, d_y, d_theta, nb_steps)

        trajectory = walk.replan(trajectory, T)
        t = T = 0.

        # elapsed_time = time.time() - start_timer
        # print(f"Replan computation time: {elapsed_time*1e6}µs")

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
        lf = trajectory.get_T_world_left(T)
        data_left.append(lf[:3, 3])

        rf = trajectory.get_T_world_right(T)
        data_right.append(rf[:3, 3])

        data.append(
            [trajectory.com.pos(T), trajectory.com.zmp(T), trajectory.com.dcm(T)])

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
