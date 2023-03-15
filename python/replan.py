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
parser.add_argument("-t", "--torque", action="store_true",
                    help="Torque visualization")
parser.add_argument("-m", "--meshcat", action="store_true",
                    help="MeshCat visualization")
args = parser.parse_args()

# Plotting (options --graph and --torque)
nb_plotted_steps = 10

# Kick
kick = False
nb_steps_before_kick = 6

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")

# Displayed joints (if argument --torque)
displayed_joints = {"left_hip_roll", "left_hip_pitch",
                    "right_hip_roll", "right_hip_pitch"}

timeline = []

# Walk parameters
parameters = placo.HumanoidParameters()
parameters.dt = 0.025
parameters.single_support_duration = .35
parameters.double_support_duration = 0.0
parameters.startend_double_support_duration = 0.5
parameters.kick_duration = 0.3
parameters.maximum_steps = 500
parameters.walk_com_height = 0.32
parameters.walk_foot_height = 0.04
parameters.pendulum_height = 0.32
parameters.walk_trunk_pitch = 0.2
parameters.walk_foot_tilt = 0.2
parameters.foot_length = 0.1576
parameters.foot_width = 0.092
parameters.feet_spacing = 0.122
parameters.zmp_margin = 0.02

# Creating the kinematics solver
solver = robot.make_solver()
task_holder = placo.SolverTaskHolder(robot, solver)

elbow = -120*np.pi/180
task_holder.update_arms_task(elbow, elbow, 0, 0, 0, 0, False)
task_holder.update_head_task(0., 0., False)

# Creating the FootstepsPlanners
T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
T_world_right = placo.flatten_on_floor(robot.get_T_world_right())

naive_footsteps_planner = placo.FootstepsPlannerNaive(parameters)
T_world_leftTarget = T_world_left.copy()
T_world_rightTarget = T_world_right.copy()
T_world_leftTarget[0, 3] += .5
T_world_rightTarget[0, 3] += .5
naive_footsteps_planner.configure(T_world_leftTarget, T_world_rightTarget)

repetitive_footsteps_planner = placo.FootstepsPlannerRepetitive(parameters)
d_x = 0.1
d_y = 0.
d_theta = 0.
nb_steps = 5
repetitive_footsteps_planner.configure(d_x, d_y, d_theta, nb_steps)

# Creating the pattern generator
walk = placo.WalkPatternGenerator(robot, parameters)

# --------------------------------------
# footsteps = naive_footsteps_planner.plan(placo.HumanoidRobot_Side.left,
#                                          T_world_left, T_world_right)
# --------------------------------------
footsteps = repetitive_footsteps_planner.plan(placo.HumanoidRobot_Side.left,
                                              T_world_left, T_world_right)
# --------------------------------------

trajectory = walk.plan(footsteps, np.array([0., 0.]), np.array([0., 0.]))

if args.graph:
    data_left = []
    data_right = []
    data = []
    draw_footsteps(trajectory.supports, show=False)

if args.pybullet or args.torque:
    import pybullet as p
    from onshape_to_robot.simulation import Simulation
    sim = Simulation("sigmaban/robot.urdf", realTime=True, dt=0.005)

    if args.torque:
        import matplotlib.pyplot as plt
        torques = {joint: [] for joint in displayed_joints}

if args.meshcat:
    viz = robot_viz(robot)
    footsteps_viz(trajectory.supports)

start_t = time.time()
t = -5.
dt = 0.005
real_time_offset = 0
steps = 0
replan = True

while True:
    T = max(0, t)
    timeline.append(t + real_time_offset)

    task_holder.update_walk_tasks(trajectory.get_T_world_left(T), trajectory.get_T_world_right(T),
                                  trajectory.get_CoM_world(T), trajectory.get_R_world_trunk(T), False)

    previous_support = robot.get_support_side()

    robot.update_support_side(str(trajectory.support_side(T)))

    # Replan
    if replan and previous_support != robot.get_support_side() and str(previous_support) != "both":
        print("\n----- Replanning -----")
        steps += 1

        if (not kick) or steps < nb_steps_before_kick:
            print("Replanning a new walk trajectory")
            # start_timer = time.time()

            T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
            T_world_right = placo.flatten_on_floor(robot.get_T_world_right())

            # --------------------------------------
            # T_world_leftTarget[0, 3] += 0.08
            # T_world_rightTarget[0, 3] += 0.08
            # naive_footsteps_planner.configure(T_world_leftTarget, T_world_rightTarget)
            # footsteps = naive_footsteps_planner.plan(placo.HumanoidRobot_Side.left,
            #                                         T_world_left, T_world_right)
            # --------------------------------------
            footsteps = repetitive_footsteps_planner.plan(placo.HumanoidRobot.other_side(previous_support),
                                                          T_world_left, T_world_right)
            # --------------------------------------

            trajectory = walk.plan(
                footsteps, trajectory.com.vel(T), trajectory.com.acc(T))

            # elapsed_time = time.time() - start_timer
            # print(f"Replan computation time: {elapsed_time*1e6}µs")

        elif steps == nb_steps_before_kick:
            print("Prepare for kicking")
            print("Kicking side : ", previous_support)

            replan = False

            T_world_target = placo.flatten_on_floor(robot.get_T_world_left()) if str(
                previous_support) == "left" else placo.flatten_on_floor(robot.get_T_world_right())

            T_world_target[2, 3] += parameters.walk_foot_height

            # trajectory = walk.plan_kick(
            #     trajectory, T, previous_support, T_world_target)

            trajectory = walk.plan_one_foot_balance(
                trajectory, T, previous_support, T_world_target)

            # task_holder.configure_weight(1., 1., 1., 0.)
            # solver.add_centroidal_momentum_task(np.array([0., 0., 0.]))

        real_time_offset += T
        t = T = 0.

    if (args.pybullet or args.torque) and t < -2:
        T_left_origin = sim.transformation("origin", "left_foot_frame")
        T_world_left = sim.poseToMatrix(([0., 0., 0.05], [0., 0., 0., 1.]))
        T_world_origin = T_world_left @ T_left_origin

        sim.setRobotPose(*sim.matrixToPose(T_world_origin))

    if args.meshcat:
        viz.display(robot.state.q)
        robot_frame_viz(robot, "left_foot")
        robot_frame_viz(robot, "right_foot")
        com = robot.com_world()
        com[2] = 0
        point_viz("com", com)

    if args.pybullet or args.torque:
        joints = {joint: robot.get_joint(joint)
                  for joint in sim.getJoints()}
        applied = sim.setJoints(joints)
        sim.tick()

        # For torques info display
        if args.torque and t >= 0:
            for joint in displayed_joints:
                torques[joint].append(applied[joint][-1])

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

    if args.graph and steps == nb_plotted_steps:
        data_left = np.array(data_left)
        data_right = np.array(data_right)
        data = np.array(data)

        plt.plot(data.T[0][0], data.T[1][0], label="CoM", lw=3)
        plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
        plt.plot(data.T[0][2], data.T[1][2], label="DCM", lw=3)

        plt.legend()
        plt.grid()
        plt.show()

    if args.graph and (T > trajectory.duration):
        data_left = np.array(data_left)
        data_right = np.array(data_right)
        data = np.array(data)

        # plt.plot(timeline, data.T[0][0], label="CoM_x", lw=3)
        # plt.plot(timeline, data.T[1][0], label="CoM_y", lw=3)

        plt.plot(data.T[0][0], data.T[1][0], label="CoM", lw=3)
        plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
        plt.plot(data.T[0][2], data.T[1][2], label="DCM", lw=3)

        draw_footsteps(trajectory.supports, False, False)

        plt.legend()
        plt.grid()
        plt.show()

    # Displaying torques info
    if args.torque and steps == nb_plotted_steps:
        for joint in displayed_joints:
            data = np.array(torques[joint])

            # Filtering false torques due to instantaneous change of position
            for i in range(1, data.size):
                if abs(data[i] - data[i-1]) > 2:
                    data[i] = data[i-1]

            plt.plot(np.array(timeline), data, label=joint)
            print(joint + " max torque value : " +
                  str(max(max(data), -min(data))) + " N.m")

        plt.legend()
        plt.show()
