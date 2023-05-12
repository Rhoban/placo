import time
import placo
import argparse
import eigenpy
import pinocchio as pin
import numpy as np
from placo_utils.tf import tf

parser = argparse.ArgumentParser(description="Process some integers.")
parser.add_argument("-g", "--graph", action="store_true", help="Plot")
parser.add_argument("-p", "--pybullet", action="store_true",
                    help="PyBullet visualization")
parser.add_argument("-m", "--meshcat", action="store_true",
                    help="MeshCat visualization")
args = parser.parse_args()

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")

# Walk parameters
parameters = placo.HumanoidParameters()
parameters.single_support_duration = 0.35
parameters.single_support_timesteps = 12
parameters.double_support_ratio = 0.0
parameters.startend_double_support_ratio = 1.5
parameters.kick_support_ratio = 2
parameters.planned_timesteps = 500
parameters.replan_timesteps = 10
parameters.walk_com_height = 0.32
parameters.walk_foot_height = 0.04
parameters.pendulum_height = 0.32
parameters.walk_trunk_pitch = 0.2
parameters.walk_foot_tilt = 0.2
parameters.foot_length = 0.1576
parameters.foot_width = 0.092
parameters.feet_spacing = 0.122
parameters.zmp_margin = 0.02
parameters.foot_zmp_target_x = -0.005
parameters.foot_zmp_target_y = 0
parameters.kick_zmp_target_x = -0.01
parameters.kick_zmp_target_y = -0.01

# Creating the kinematics solver
solver = robot.make_solver()

robot.set_velocity_limits(5.)
solver.enable_velocity_limits(True)
solver.dt = 0.005

T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
T_world_right = placo.flatten_on_floor(robot.get_T_world_right())

tasks = placo.WalkTasks()
tasks.initialize_tasks(solver)

elbow = -120*np.pi/180
joints_task = solver.add_joints_task()
joints_task.set_joints({
    "left_shoulder_roll": 0.,
    "left_shoulder_pitch": 0.,
    "left_elbow": elbow,
    "right_shoulder_roll": 0.,
    "right_shoulder_pitch": 0.,
    "right_elbow": elbow,
    "head_pitch": 0.,
    "head_yaw": 0.
})
joints_task.configure("joints", "soft", 1.)

solver.add_regularization_task(1e-6)

robot.update_kinematics()
solver.solve(True)

repetitive_footsteps_planner = placo.FootstepsPlannerRepetitive(parameters)
d_x = 0.15
d_y = 0.
d_theta = 0.
nb_steps = 5
repetitive_footsteps_planner.configure(d_x, d_y, d_theta, nb_steps)

footsteps = repetitive_footsteps_planner.plan(placo.HumanoidRobot_Side.left,
                                              T_world_left, T_world_right)

supports = placo.FootstepsPlanner.make_supports(
    footsteps, True, parameters.has_double_support(), True)

# Kick on the last support before double support
supports[nb_steps].kick = True

# Creating the walk pattern generator and planification
walk = placo.WalkPatternGenerator(robot, parameters)
trajectory = walk.plan(supports, 0.)

if args.graph:
    import matplotlib.pyplot as plt
    from footsteps_planner import draw_footsteps

    ts = np.linspace(0, trajectory.t_end, 1000)
    data_left = []
    data_right = []

    for t in ts:
        T = trajectory.get_T_world_left(t)
        data_left.append(T[:3, 3])

        T = trajectory.get_T_world_right(t)
        data_right.append(T[:3, 3])

    data = np.array([[trajectory.com.pos(t), trajectory.com.zmp(
        t), trajectory.com.dcm(t), trajectory.com.jerk(t)] for t in ts])

    data_left = np.array(data_left)
    data_right = np.array(data_right)

    draw_footsteps(trajectory.supports, show=False)

    for t in np.linspace(0, trajectory.t_end, 100):
        x_values = np.array(
            [trajectory.com.pos(t)[0], trajectory.com.dcm(t)[0]])
        y_values = np.array(
            [trajectory.com.pos(t)[1], trajectory.com.dcm(t)[1]])
        plt.plot(x_values, y_values, c="grey")

    plt.plot(data.T[0][0], data.T[1][0], label="CoM", c="red", lw=3)
    plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
    plt.plot(data.T[0][2], data.T[1][2], label="DCM", c="green", lw=3)
    # plt.plot(data.T[0][3], data.T[1][3], label="Jerk", c="orange")

    plt.legend()
    plt.grid()
    plt.show()

elif args.pybullet or args.meshcat:
    from placo_utils.visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, footsteps_viz

    if args.pybullet:
        import pybullet as p
        from onshape_to_robot.simulation import Simulation
        sim = Simulation("sigmaban/robot.urdf", realTime=True, dt=0.005)

    if args.meshcat:
        viz = robot_viz(robot)
        footsteps_viz(trajectory.supports)

    start_t = time.time()
    t = -3. if args.pybullet or args.meshcat else 0.
    dt = 0.005
    last_display = 0

    while True:
        T = max(0, t)
        if T > trajectory.t_end:
            continue

        tasks.update_tasks_from_trajectory(trajectory, T)
        robot.update_kinematics()
        solver.solve(True)

        if not trajectory.support_is_both(T):
            robot.update_support_side(str(trajectory.support_side(T)))
            robot.ensure_on_floor()

        if args.pybullet and t < -2:
            T_left_origin = sim.transformation("origin", "left_foot_frame")
            T_world_left = sim.poseToMatrix(([0., 0., 0.05], [0., 0., 0., 1.]))
            T_world_origin = T_world_left @ T_left_origin

            sim.setRobotPose(*sim.matrixToPose(T_world_origin))

        if args.meshcat:
            if time.time() - last_display > 0.04:
                last_display = time.time()
                viz.display(robot.state.q)

            # robot_frame_viz(robot, "left_foot")
            # robot_frame_viz(robot, "right_foot")
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
        while time.time() < start_t + t:
            time.sleep(1e-3)
        time.sleep(dt)
