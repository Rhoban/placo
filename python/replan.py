import time
import placo
import argparse
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import warnings
from footsteps_planner import draw_footsteps
from placo_utils.visualization import robot_viz, frame_viz, point_viz, robot_frame_viz, footsteps_viz

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
parameters.single_support_duration = 0.35
parameters.single_support_timesteps = 12
parameters.double_support_ratio = 0.0
parameters.startend_double_support_ratio = 1.5
parameters.planned_timesteps = 64
parameters.kick_duration = 0.3
parameters.replan_timesteps = 6
parameters.walk_com_height = 0.32
parameters.walk_foot_height = 0.04
parameters.pendulum_height = 0.32
parameters.walk_trunk_pitch = 0.2
parameters.walk_foot_tilt = 0.2
parameters.foot_length = 0.1576
parameters.foot_width = 0.092
parameters.feet_spacing = 0.122
parameters.zmp_margin = 0.02


def plot_CoM(trajectory, t_replan=0.0, plot_time=0.8):
    data = []
    tl = np.linspace(0, parameters.planned_timesteps * parameters.dt())

    for ti in tl:
        data.append(
            [trajectory.com.pos(ti + t_replan), trajectory.com.zmp(ti + t_replan), trajectory.com.dcm(ti + t_replan)])

    data = np.array(data)

    plt.plot(data.T[0][0], data.T[1][0], label="CoM", lw=3)
    plt.plot(data.T[0][1], data.T[1][1], label="ZMP", lw=3)
    plt.plot(data.T[0][2], data.T[1][2], label="DCM", lw=3)

    draw_footsteps(trajectory.supports, show=False)

    plt.legend()
    plt.grid()
    plt.xlim([-.2, 1.5])
    plt.ylim([-.2, .13])
    plt.show(block=False)
    plt.pause(plot_time)
    plt.close()


# Creating the kinematics solver
solver = robot.make_solver()
tasks = placo.WalkTasks()
tasks.initialize_tasks(solver)

elbow = -120*np.pi/180
joints_task = solver.add_joints_task()
joints_task.set_joints({
    "left_shoulder_roll": 0.,
    "left_shoulder_pitch": 0.,
    "left_elbow": -1.0,
    "right_shoulder_roll": 0.,
    "right_shoulder_pitch": 0.,
    "right_elbow": -1.0,
    "head_pitch": 0.,
    "head_yaw": 0.
})
joints_task.configure("joints", "soft", 1.)

solver.add_regularization_task(1e-6)

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
d_y = 0.0
d_theta = 0.
nb_steps = 10
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

supports = placo.FootstepsPlanner.make_supports(
    footsteps, True, parameters.has_double_support(), True)

trajectory = walk.plan(supports, 0.)

if args.graph:
    plot_CoM(trajectory)

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
adapting_trajectory_time = 1.3
coef = 1
last_replan = 0
last_display = 0

while True:
    T = min(trajectory.t_end, max(0, t))

    robot.update_kinematics()
    tasks.update_tasks(trajectory, T)
    solver.solve(True)

    if not trajectory.is_both_support(T):
        robot.update_support_side(str(trajectory.support_side(T)))
        robot.ensure_on_floor()

    # Replanning
    if T - last_replan > parameters.replan_timesteps * parameters.dt() and walk.can_replan_supports(trajectory, T):
        last_replan = T
        supports = walk.replan_supports(
            repetitive_footsteps_planner, trajectory, T)

        if args.meshcat:
            footsteps_viz(supports)

        trajectory = walk.replan(supports, trajectory, T)

        if args.graph:
            plot_CoM(trajectory, T)

    if args.pybullet and t < -2:
        T_left_origin = sim.transformation("origin", "left_foot_frame")
        T_world_left = sim.poseToMatrix(([0., 0., 0.05], [0., 0., 0., 1.]))
        T_world_origin = T_world_left @ T_left_origin

        sim.setRobotPose(*sim.matrixToPose(T_world_origin))

    if args.meshcat:
        if time.time() - last_display > 0.03:
            last_display = time.time()
            viz.display(robot.state.q)
            # robot_frame_viz(robot, "left_foot")
            # robot_frame_viz(robot, "right_foot")
            com = robot.com_world()
            com[2] = 0
            point_viz("com", com)

            frame_viz("left_foot_target", trajectory.get_T_world_left(T))
            frame_viz("right_foot_target", trajectory.get_T_world_right(T))

            T_world_trunk = np.eye(4)
            T_world_trunk[:3, :3] = trajectory.get_R_world_trunk(T)
            T_world_trunk[:3, 3] = trajectory.get_p_world_CoM(T)
            frame_viz("trunk_target", T_world_trunk)

    if args.pybullet:
        joints = {joint: robot.get_joint(joint)
                  for joint in sim.getJoints()}
        applied = sim.setJoints(joints)
        sim.tick()

    # Spin-lock until the next tick
    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
    # time.sleep(1e-2)
