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

# Displayed joints (if argument --torque)
displayed_joints = {"left_hip_roll", "left_hip_pitch",
                    "right_hip_roll", "right_hip_pitch"}

# Walk parameters
parameters = placo.HumanoidParameters()
parameters.single_support_duration = 0.35
parameters.single_support_timesteps = 12
parameters.double_support_ratio = 0.0
parameters.startend_double_support_ratio = 1.5
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
parameters.zmp_margin = 0.0

# Creating the kinematics solver
solver = robot.make_solver()

robot.set_velocity_limits(5.)
solver.enable_velocity_limits(True)
solver.dt = 0.005

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

# Initializing the walk
repetitive_footsteps_planner = placo.FootstepsPlannerRepetitive(parameters)
d_x = 0.1
d_y = 0.
d_theta = 0.
nb_steps = 3
repetitive_footsteps_planner.configure(d_x, d_y, d_theta, nb_steps)

T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
T_world_right = placo.flatten_on_floor(robot.get_T_world_right())
footsteps = repetitive_footsteps_planner.plan(placo.HumanoidRobot_Side.left,
                                              T_world_left, T_world_right)

supports = placo.FootstepsPlanner.make_supports(
    footsteps, True, parameters.has_double_support(), True)

walk = placo.WalkPatternGenerator(robot, parameters)
trajectory = walk.plan(supports, 0.)

# Creating the Kick
kick = placo.Kick(robot, parameters)
kicking_side = placo.HumanoidRobot_Side.left
support_side = placo.HumanoidRobot.other_side(kicking_side)

kick.t_init = 0.7
kick.t_delay = 0.3
kick.t_up = 0.3

ampl_sin = 0.1
init_sin = False

kicking = False

if args.pybullet or args.meshcat:
    from placo_utils.visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, footsteps_viz

    if args.pybullet:
        import pybullet as p
        from onshape_to_robot.simulation import Simulation
        sim = Simulation("sigmaban/robot.urdf", realTime=True, dt=0.005)

    if args.meshcat:
        viz = robot_viz(robot)

    start_t = time.time()
    t = -1. if args.pybullet or args.meshcat else 0.
    dt = 0.005
    last_display = 0

    while True:
        T = max(0, t)
        if not kicking:
            if T > trajectory.t_end:

                kick.one_foot_balance(
                    repetitive_footsteps_planner, support_side)

                robot.update_support_side(str(support_side))
                kicking = True
                t = 0.
                continue

            tasks.update_tasks_from_trajectory(trajectory, T)

        else:
            if T > kick.duration:
                if not init_sin:
                    init_T_world_left = robot.get_T_world_left()
                    init_T_world_right = robot.get_T_world_right()
                    init_com_world = robot.com_world()
                    init_R_world_trunk = robot.get_T_world_trunk()[:3, :3]
                    init_sin = True

                T -= kick.duration

                T_world_left = init_T_world_left.copy()
                T_world_right = init_T_world_right.copy()

                if kicking_side == placo.HumanoidRobot_Side.left:
                    T_world_left[:3, 3] += T_world_left[:3, :3] \
                        @ np.array([ampl_sin * np.sin(2*T), 0., 0.])
                else:
                    T_world_right[:3, 3] += T_world_right[:3, :3] \
                        @ np.array([ampl_sin * np.sin(2*T), 0., 0.])

                tasks.update_tasks(T_world_left, T_world_right,
                                   init_com_world, init_R_world_trunk)

            else:
                tasks.update_tasks_from_kick(kick, T)

        robot.update_kinematics()
        solver.solve(True)

        if not kicking and not trajectory.support_is_both(T):
            robot.update_support_side(str(trajectory.support_side(T)))
        robot.ensure_on_floor()

        if args.pybullet and t < -0.5:
            T_left_origin = sim.transformation("origin", "left_foot_frame")
            T_world_left = sim.poseToMatrix(([0., 0., 0.05], [0., 0., 0., 1.]))
            T_world_origin = T_world_left @ T_left_origin

            sim.setRobotPose(*sim.matrixToPose(T_world_origin))

        if args.meshcat:
            if time.time() - last_display > 0.04:
                last_display = time.time()
                viz.display(robot.state.q)

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
