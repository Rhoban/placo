import time
import placo
import argparse
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import warnings
from footsteps_planner import draw_footsteps
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, footsteps_viz

warnings.filterwarnings("ignore")

parser = argparse.ArgumentParser(description="Process some integers.")
parser.add_argument("-g", "--graph", action="store_true", help="Plot")
parser.add_argument("-p", "--pybullet", action="store_true",
                    help="PyBullet visualization")
parser.add_argument("-m", "--meshcat", action="store_true",
                    help="MeshCat visualization")
args = parser.parse_args()

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")

# Walk parameters - if double_support_duration is not set to 0, should be greater than replan_frequency * dt
parameters = placo.HumanoidParameters()
parameters.dt = 0.025
parameters.single_support_duration = .3
parameters.double_support_duration = 0.0
parameters.startend_double_support_duration = 0.35
parameters.kick_duration = 0.3
parameters.planned_dt = 64
parameters.replan_frequency = 9
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
task_holder.update_arms_task(elbow, elbow, 0., 0., 0., 0.)
task_holder.update_head_task(0., 0.)

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
nb_steps = 6
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

double_supports = parameters.double_support_duration / parameters.dt >= 1
supports = placo.FootstepsPlanner.make_supports(
    footsteps, True, double_supports, True)

trajectory = walk.plan(supports)

if args.graph:
    data = []
    tl = np.linspace(0, parameters.planned_dt * parameters.dt)

    for ti in tl:
        data.append(
            [trajectory.com.pos(ti), trajectory.com.zmp(ti), trajectory.com.dcm(ti)])

    data = np.array(data)

    plt.plot(data.T[0][0], data.T[1][0], label="CoM", lw=3)
    plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
    plt.plot(data.T[0][2], data.T[1][2], label="DCM", lw=3)

    draw_footsteps(trajectory.supports, show=False)

    plt.legend()
    plt.grid()
    # plt.show()
    plt.show(block=False)
    plt.pause(0.8)
    plt.close()

if args.pybullet:
    import pybullet as p
    from onshape_to_robot.simulation import Simulation
    sim = Simulation("sigmaban/robot.urdf", realTime=True, dt=0.005)

if args.meshcat:
    viz = robot_viz(robot)
    footsteps_viz(trajectory.supports)

start_t = time.time()
t = -3
dt = 0.005
real_time = t
adapting_trajectory_time = 1.3
coef = 1

while True:
    T = max(0, t)
    if T > trajectory.duration - trajectory.time_offset:
        continue

    # Changing the trajectory of the robot
    if real_time > adapting_trajectory_time:
        if trajectory.are_supports_updatable:
            print("Supports updated!")
            adapting_trajectory_time += 1.5
            # coef = -coef

            d_x = 0.05
            d_y = 0.
            d_theta = coef * 0.3
            nb_steps = 20
            repetitive_footsteps_planner.configure(d_x, d_y, d_theta, nb_steps)

            current_support = trajectory.get_support(T)
            next_support = trajectory.get_next_support(T)
            prev_support = trajectory.get_prev_support(T)

            flying_side = current_support.side()
            print("side :", flying_side)

            if flying_side == placo.HumanoidRobot_Side.left:
                T_world_left = current_support.frame()
                T_world_right = next_support.footstep_frame(
                    placo.HumanoidRobot_Side.right)

            else:
                T_world_right = current_support.frame()
                T_world_left = next_support.footstep_frame(
                    placo.HumanoidRobot_Side.left)

            footsteps = repetitive_footsteps_planner.plan(
                flying_side, T_world_left, T_world_right)

            # for footstep in footsteps:
            #     print("---------------------------")
            #     print("footstep side :", footstep.side)

            if double_supports:
                supports = placo.FootstepsPlanner.make_supports(
                    footsteps, True, True, True)

                placo.FootstepsPlanner.add_first_support(
                    supports, current_support)

            else:
                supports = placo.FootstepsPlanner.make_supports(
                    footsteps, False, False, True)

            # x = []
            # y = []
            # for support in supports:
            #     print("---------------------------")
            #     print("size :", len(support.footsteps))
            #     print("start :", support.start)
            #     print("end :", support.end)
            #     print(support.frame()[0, 3])
            #     x.append(support.frame()[0, 3])
            #     print(support.frame()[1, 3])
            #     y.append(support.frame()[1, 3])
            # x = np.array(x)
            # y = np.array(y)
            # plt.scatter(x, y)
            # plt.show()

            trajectory.set_supports_update_offset(
                trajectory.get_phase_t_start(T))
            trajectory.set_initial_T_world_flying_foot(
                prev_support.footstep_frame(placo.HumanoidRobot.other_side(flying_side)))

            if args.meshcat:
                footsteps_viz(trajectory.supports)

        else:
            print("Supports update delayed - wrong timing")
            adapting_trajectory_time += parameters.dt

    task_holder.update_walk_tasks(trajectory.get_T_world_left(T), trajectory.get_T_world_right(T),
                                  trajectory.get_p_world_CoM(T), trajectory.get_R_world_trunk(T), False)

    robot.update_support_side(str(trajectory.support_side(T)))
    robot.ensure_on_floor()

    # Replanning
    if walk.replan(supports, trajectory, T):
        print("Trajectory replanned")
        T = t = 0

        if args.graph:
            data = []
            tl = np.linspace(0, parameters.planned_dt * parameters.dt)

            for ti in tl:
                data.append(
                    [trajectory.com.pos(ti), trajectory.com.zmp(ti), trajectory.com.dcm(ti)])

            data = np.array(data)

            plt.plot(data.T[0][0], data.T[1][0], label="CoM", lw=3)
            plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
            plt.plot(data.T[0][2], data.T[1][2], label="DCM", lw=3)

            draw_footsteps(trajectory.supports, show=False)

            plt.legend()
            plt.grid()
            # plt.show()
            plt.show(block=False)
            plt.pause(1)
            plt.close()

    if args.pybullet and t < -2:
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

    if args.pybullet:
        joints = {joint: robot.get_joint(joint)
                  for joint in sim.getJoints()}
        applied = sim.setJoints(joints)
        sim.tick()

    # Spin-lock until the next tick
    t += dt
    real_time += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
