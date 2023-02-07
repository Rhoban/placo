import time
import placo
import argparse
import tf
import pinocchio as pin
import numpy as np

# XXX: Have the three options (plot / meshcat / pybullet) available
# XXX: Make double support? an option of the walk (maybe false if the length is zero)
# XXX: Make the constraint "duration" an option of the walk

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

# TO DO - ADD A WAY TO ADD TASK IN THE TASK_HOLDER
# joints = solver.add_joints_task()
# joints.set_joints(
#     {
#         "head_pitch": 0,
#         "head_yaw": 0,
#         "left_elbow": -2,
#         "right_elbow": -2,
#         "left_shoulder_pitch": 0,
#         "right_shoulder_pitch": 0,
#         "left_shoulder_roll": 0,
#         "right_shoulder_roll": 0,
#     }
# )

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

# start_t = time.time()

# Planification
trajectory = walk.plan()

# elapsed = time.time() - start_t
# print(
#     f"Computation time: {elapsed*1e6}µs, Jerk planner steps: {trajectory.jerk_planner_steps}")

if args.graph:
    import matplotlib.pyplot as plt
    from footsteps_planner import draw_footsteps

    ts = np.linspace(0, trajectory.duration, 1000)
    data_left = []
    data_right = []

    for t in ts:
        T = trajectory.get_T_world_left(t)
        data_left.append(T[:3, 3])

        T = trajectory.get_T_world_right(t)
        data_right.append(T[:3, 3])

    data = np.array([[trajectory.com.pos(t), trajectory.com.zmp(
        t), trajectory.com.dcm(t)] for t in ts])

    data_left = np.array(data_left)
    data_right = np.array(data_right)

    # Plotting left foot X/Y
    # plt.plot(data_left.T[0], data_left.T[2])
    # plt.grid()
    # plt.show()

    draw_footsteps(trajectory.supports, show=False)
    plt.plot(data.T[0][0], data.T[1][0], label="CoM", lw=3)
    plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
    plt.plot(data.T[0][2], data.T[1][2], label="DCM", lw=3)
    plt.legend()
    plt.grid()
    plt.show()

elif args.pybullet or args.meshcat:
    from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, footsteps_viz

    if args.pybullet:
        import pybullet as p
        from onshape_to_robot.simulation import Simulation
        sim = Simulation("sigmaban/robot.urdf", realTime=True, dt=0.005)

    if args.meshcat:
        viz = robot_viz(robot)
        footsteps_viz(trajectory.supports)

    start_t = time.time()
    t = -1.5 if args.pybullet or args.meshcat else 0.
    dt = 0.005

    while True:
        T = max(0, t)

        task_holder.update_tasks(trajectory.get_T_world_left(T), trajectory.get_T_world_right(T),
                                 trajectory.get_CoM_world(T), trajectory.get_R_world_trunk(T), False)

        robot.update_kinematics()
        robot.update_support_side(str(trajectory.support_side(T)))

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
        while time.time() < start_t + t:
            time.sleep(1e-3)
