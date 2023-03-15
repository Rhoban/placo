import time
import placo
import numpy as np
import argparse
from onshape_to_robot.simulation import Simulation
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz

parser = argparse.ArgumentParser(description="Process some integers.")
parser.add_argument("-p", "--pybullet", action="store_true",
                    help="PyBullet visualization")
parser.add_argument("-m", "--meshcat", action="store_true",
                    help="MeshCat visualization")
parser.add_argument("-g", "--graph", action="store_true",
                    help="Graph plot")
args = parser.parse_args()

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")

# robot.set_joint("left_knee", 0.1)
# robot.set_joint("right_knee", 0.1)
# robot.update_kinematics()
robot.set_T_world_frame("left_foot", np.eye(4))
robot.update_kinematics()

solver = robot.make_solver()

# Retrieving initial position of the feet, com and trunk orientation
T_world_left = robot.get_T_world_frame("left_foot")
T_world_right = robot.get_T_world_frame("right_foot")

# Keep left and right foot on the floor
left_foot_task = solver.add_frame_task("left_foot", T_world_left)
left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)
right_foot_task.T_world_frame = T_world_right

# Trunk position
T_world_frame = robot.get_T_world_frame("trunk")

trunk_task = solver.add_orientation_task("trunk", np.eye(3))
trunk_task.configure("trunk_task", "soft", 1.0)

com = np.array([0., -0.05, 0.3])
com_task = solver.add_com_task(com)
com_task.configure("com", "soft", 1.0)

# Setting custom target values for elbows
joints_task = solver.add_joints_task()
joints_task.configure("joints", "soft", 1.0)

joints_task.set_joints({
    "left_shoulder_roll": 0.,
    "left_elbow": -2.0,
    "right_shoulder_roll": 0.,
    "right_elbow": -2.0,
    "head_pitch": 0.,
    "head_yaw": 0.
})

shoulder_pitchs_tasks = joints_task = solver.add_joints_task()
shoulder_pitchs_tasks.configure("joints", "soft", 1e-6)

shoulder_pitchs_tasks.set_joints({
    "left_shoulder_pitch": 0.,
    "right_shoulder_pitch": 0.,
})


solver.add_regularization_task(1e-6)

foot_spline = placo.PolySpline3D()
foot_spline.addPoint(0., np.array([0., -0.1, 0.]), np.array([0., 0., 0.]))
foot_spline.addPoint(0.5, np.array([0., -0.1, 0.]), np.array([0., 0., 0.]))
foot_spline.addPoint(1., np.array([0., -0.1, 0.05]), np.array([0., 0., 0.]))
foot_spline.addPoint(1.5, np.array(
    [-0.08, -0.1, 0.05]), np.array([0., 0., 0.]))
foot_spline.addPoint(1.51, np.array(
    [0.08, -0.1, 0.05]), np.array([0., 0., 0.]))
foot_spline.addPoint(2., np.array([0.0, -0.1, 0.05]), np.array([0., 0., 0.]))
foot_spline.addPoint(2.5, np.array([0.0, -0.1, 0.]), np.array([0., 0., 0.]))

T_world_right[:3, 3] = foot_spline.get(0)

com_spline = placo.PolySpline()
com_spline.addPoint(0., -0.05, 0)
com_spline.addPoint(1, 0., 0)
com_spline.addPoint(2.5, 0., 0)
com_spline.addPoint(3.5, -0.05, 0)
com[1] = com_spline.get(0)

rising = False

for _ in range(16):
    robot.update_kinematics()
    qd = solver.solve(True)

if args.meshcat:
    # Creating the viewer
    viz = robot_viz(robot)

if args.pybullet:
    import pybullet as p
    from onshape_to_robot.simulation import Simulation
    sim = Simulation("sigmaban/robot.urdf", realTime=True, dt=0.005)

    T_left_origin = sim.transformation("origin", "left_foot_frame")
    T_world_origin = T_world_left @ T_left_origin

    sim.setRobotPose(*sim.matrixToPose(T_world_origin))

if args.graph:
    times_plot = []
    joints_plot = {joint: [] for joint in robot.actuated_joint_names()}

t = 0
dt = 0.005
start_t = time.time()
robot.update_kinematics()
start_motion = 4.

try:
    for step in range(int(1e9)):
        w = 1
        motion_t = max(0, t-start_motion)

        if True:
            t0 = time.time()

            if motion_t > 0.:
                if not rising:
                    rising = True
                    trunk_task.configure("trunk_task", "soft", 1e-4)
                    solver.add_centroidal_momentum_task(np.array([0., 0., 0.]))

            T_world_right[:3, 3] = foot_spline.get(motion_t)
            com[1] = com_spline.get(motion_t)

            right_foot_task.T_world_frame = T_world_right
            com_task.target_world = com

            qd = solver.solve(True)
            elapsed = time.time() - t0

            print(f"Computation time: {elapsed*1e6}µs")
            robot.update_kinematics()
            solver.dump_status()

        t += dt
        if args.meshcat:
            if step % 10 == 0:
                viz.display(robot.state.q)
                # Show some frames
                robot_frame_viz(robot, "camera")
                robot_frame_viz(robot, "left_foot")
                robot_frame_viz(robot, "right_foot")
                robot_frame_viz(robot, "trunk")

                # Show the CoM
                point_viz("com", robot.com_world(),
                          radius=0.025, color=0xAAAAAA)

            # Spin-lock until the next tick
            while time.time() < start_t + t:
                time.sleep(1e-3)

        if args.pybullet:
            joints = {joint: robot.get_joint(joint)
                      for joint in sim.getJoints()}
            applied = sim.setJoints(joints)
            sim.tick()

        if args.graph:
            times_plot.append(t)
            for joint in robot.actuated_joint_names():
                joints_plot[joint].append(robot.get_joint(joint))

except KeyboardInterrupt:
    if args.graph:
        import matplotlib.pyplot as plt

        for joint in robot.actuated_joint_names():
            plt.plot(times_plot, joints_plot[joint], label=joint)

        plt.legend()
        plt.grid()
        plt.show()
