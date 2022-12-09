import time
import placo
import argparse
import pinocchio as pin
import numpy as np

# XXX: Have the three options (plot / meshcat / pybullet) available
# XXX: Make double support? an option of the walk (maybe false if the length is zero)
# XXX: Make the constraint "duration" an option of the walk

parser = argparse.ArgumentParser(description="Process some integers.")
parser.add_argument("-g", "--graph", action="store_true", help="Plot")
parser.add_argument("-p", "--pybullet", action="store_true", help="PyBullet visualization")
parser.add_argument("-m", "--meshcat", action="store_true", help="MeshCat visualization")
args = parser.parse_args()

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")
robot.ensure_on_floor()

# Planning the steps
T_world_left = robot.get_T_world_left()
T_world_right = robot.get_T_world_right()

T_world_leftTarget = T_world_left.copy()
T_world_rightTarget = T_world_right.copy()

T_world_leftTarget[0, 3] += 1.0
T_world_rightTarget[0, 3] += 1.0
# T_world_leftTarget[1, 3] += 1.5
# T_world_rightTarget[1, 3] += 1.5

# Creating the pattern generator to plan the trajectory
walk = placo.WalkPatternGenerator(robot)
walk.parameters.single_support_duration = 0.4
walk.parameters.double_support_duration = 0.1
walk.parameters.startend_double_support_duration = 0.5
walk.parameters.maximum_steps = 1024
walk.parameters.walk_com_height = 0.32
walk.parameters.pendulum_height = 0.3
walk.parameters.walk_trunk_pitch = 0.2
walk.parameters.foot_length = 0.1576
walk.parameters.foot_width = 0.092
walk.parameters.feet_spacing = 0.122
walk.parameters.zmp_margin = 0.045

start_t = time.time()
trajectory = walk.plan(T_world_leftTarget, T_world_rightTarget)
elapsed = time.time() - start_t
print(f"Computation time: {elapsed*1e6}µs")

# Creating the kinematics solver
solver = robot.make_solver()

left_foot = solver.add_frame_task("left_foot", T_world_left)
left_foot.configure("left_foot", "soft", 1., 1.)

right_foot = solver.add_frame_task("right_foot", T_world_left)
right_foot.configure("right_foot", "soft", 1., 1.)

com_task = solver.add_com_task(np.array([0., 0., 0.]))
com_task.configure("com", "soft", 1.0)

trunk_orientation_task = solver.add_orientation_task("trunk", np.eye(3))
trunk_orientation_task.configure("trunk", "soft", 1.0)

joints = solver.add_joints_task()
joints.set_joints(
    {
        "head_pitch": 0,
        "head_yaw": 0,
        "left_elbow": -2,
        "right_elbow": -2,
        "left_shoulder_pitch": 0,
        "right_shoulder_pitch": 0,
        "left_shoulder_roll": 0,
        "right_shoulder_roll": 0,
    }
)

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

    data = np.array([[trajectory.com.pos(t), trajectory.com.zmp(t), trajectory.com.dcm(t)] for t in ts])

    data_left = np.array(data_left)
    data_right = np.array(data_right)

    # Plotting left foot X/Y
    # plt.plot(data_left.T[0], data_left.T[2])
    # plt.grid()
    # plt.show()

    draw_footsteps(trajectory.footsteps, show=False)
    plt.plot(data.T[0][0], data.T[1][0], label="CoM", lw=3)
    plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
    plt.plot(data.T[0][2], data.T[1][2], label="DCM", lw=3)
    plt.legend()
    plt.grid()
    plt.show()

elif args.pybullet or args.meshcat:
    from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, footsteps_viz

    solver.add_regularization_task(1e-6)

    if args.pybullet:
        import pybullet as p
        from onshape_to_robot.simulation import Simulation
        sim = Simulation("sigmaban/robot.urdf", realTime=True, dt=0.005)

    if args.meshcat:
        viz = robot_viz(robot)
        footsteps_viz(trajectory.footsteps)

    start_t = time.time()
    t = -3. if args.pybullet else 0.
    dt = 0.005

    while True:
        robot.update_kinematics()
        T = max(0, t)

        if args.pybullet and t < -2:
            _, orn = p.getBasePositionAndOrientation(sim.robot)
            sim.setRobotPose([0., 0.,  .5], orn)

        left_foot.T_world_frame = trajectory.get_T_world_left(T)
        right_foot.T_world_frame = trajectory.get_T_world_right(T)
        com_task.target_world = trajectory.get_CoM_world(T)
        trunk_orientation_task.R_world_frame = trajectory.get_R_world_trunk(T)

        solver.solve(True)
        solver.dump_status()

        if args.meshcat:
            viz.display(robot.state.q)

            robot_frame_viz(robot, "left_foot")
            robot_frame_viz(robot, "right_foot")
            com = robot.com_world()
            com[2] = 0
            point_viz("com", com)

        if args.pybullet:
            joints = {joint: robot.get_joint(joint) for joint in sim.getJoints()}
            applied = sim.setJoints(joints)
            sim.tick()

        # Spin-lock until the next tick
        t += dt
        while time.time() < start_t + t:
            time.sleep(1e-3)
