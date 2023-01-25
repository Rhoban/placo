import time
import placo
import pinocchio as pin
import numpy as np
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, footsteps_viz

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")
robot.load()
robot.ensure_on_floor()

# Creating the pattern generator to plan the trajectory
walk = placo.WalkPatternGenerator(robot)
walk.parameters.dt = 0.025
walk.parameters.single_support_duration = .35
walk.parameters.double_support_duration = 0.
walk.parameters.startend_double_support_duration = 0.5
walk.parameters.maximum_steps = 64
walk.parameters.walk_com_height = 0.32
walk.parameters.walk_foot_height = 0.04
walk.parameters.pendulum_height = 0.32
walk.parameters.walk_trunk_pitch = 0.2
walk.parameters.walk_foot_tilt = 0.2
walk.parameters.foot_length = 0.1576
walk.parameters.foot_width = 0.092
walk.parameters.feet_spacing = 0.122
walk.parameters.zmp_margin = 0.045

# Repetitive footsteps planner parameters
d_x = 0.1
d_y = 0.
d_theta = 0.
nb_steps = 5
previous_support = "both"
previous_flying_foot = "right"
double_support = (walk.parameters.double_support_duration /
                  walk.parameters.dt) > 1

# Planning the first steps
T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
T_world_right = placo.flatten_on_floor(robot.get_T_world_right())

planner = placo.FootstepsPlannerRepetitive(
    previous_flying_foot, T_world_left, T_world_right)
planner.parameters.feet_spacing = walk.parameters.feet_spacing
planner.parameters.foot_width = walk.parameters.foot_width
planner.parameters.foot_length = walk.parameters.foot_length
footsteps = planner.plan(d_x, d_y, d_theta, nb_steps)
supports = planner.make_double_supports(footsteps, True, double_support, True)
trajectory = walk.plan_by_supports(supports)

# Creating the kinematics solver
solver = robot.make_solver()
solver.add_regularization_task(1e-6)
solver.mask_dof("head_pitch")
solver.mask_dof("head_yaw")
solver.mask_dof("left_elbow")
solver.mask_dof("right_elbow")
solver.mask_dof("left_shoulder_pitch")
solver.mask_dof("right_shoulder_pitch")
solver.mask_dof("left_shoulder_roll")
solver.mask_dof("right_shoulder_roll")

left_foot = solver.add_frame_task("left_foot", T_world_left)
left_foot.configure("left_foot", "soft", 1., 1.)

right_foot = solver.add_frame_task("right_foot", T_world_left)
right_foot.configure("right_foot", "soft", 1., 1.)

com_task = solver.add_com_task(np.array([0., 0., 0.]))
com_task.configure("com", "soft", 1.0)

trunk_orientation_task = solver.add_orientation_task("trunk", np.eye(3))
trunk_orientation_task.configure("trunk", "soft", 1.0)

# Meshcat
viz = robot_viz(robot)
footsteps_viz(trajectory.footsteps)

start_t = time.time()
t = -2.
dt = 0.005

while True:

    robot.update_kinematics()

    T = max(0, t)

    left_foot.T_world_frame = trajectory.get_T_world_left(T)
    right_foot.T_world_frame = trajectory.get_T_world_right(T)
    com_task.target_world = trajectory.get_CoM_world(T)
    trunk_orientation_task.R_world_frame = trajectory.get_R_world_trunk(T)

    solver.solve(True)
    # solver.dump_status()

    # Meshcat
    viz.display(robot.state.q)
    robot_frame_viz(robot, "left_foot")
    robot_frame_viz(robot, "right_foot")
    com = robot.com_world()
    com[2] = 0
    point_viz("com", com)

    # Replan
    if previous_support != trajectory.support_side(T) and str(previous_support) != "both":
        print("REPLAN")
        # start_t = time.time()

        last_footstep = placo.flatten_on_floor(
            trajectory.get_last_footstep(T, double_support))
        last_last_footstep = placo.flatten_on_floor(
            trajectory.get_last_last_footstep(T, double_support))

        if previous_flying_foot == "right":
            planner = placo.FootstepsPlannerRepetitive(
                "left", last_last_footstep, last_footstep)
            previous_flying_foot = "left"

        else:
            planner = placo.FootstepsPlannerRepetitive(
                "right", last_footstep, last_last_footstep)
            previous_flying_foot = "right"

        planner.parameters.feet_spacing = walk.parameters.feet_spacing
        planner.parameters.foot_width = walk.parameters.foot_width
        planner.parameters.foot_length = walk.parameters.foot_length

        footsteps = planner.plan(d_x, d_y, d_theta, nb_steps)
        supports = planner.make_double_supports(
            footsteps, double_support, double_support, True)
        trajectory = walk.plan_by_supports(supports)

        # elapsed = time.time() - start_t
        # print(f"Replan computation time: {elapsed*1e6}µs")

        t = 0
        T = 0

    previous_support = trajectory.support_side(T)

    # Spin-lock until the next tick
    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
