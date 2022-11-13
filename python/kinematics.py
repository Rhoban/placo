import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
import tf
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz

# TODO: Update joint limits in URDF

# Loading the robot
robot = placo.MobileRobot("sigmaban/")

# robot.set_joint("left_knee", 0.1)
# robot.set_joint("right_knee", 0.1)
# robot.update_kinematics()
robot.set_T_world_frame("left_foot_tip", np.eye(4))
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)

# Retrieving initial position of the feet, com and trunk orientation
T_world_left = robot.get_T_world_frame("left_foot_tip")
T_world_right = robot.get_T_world_frame("right_foot_tip")

# Creating the viewer
viz = robot_viz(robot)

# Keep left and right foot on the floor
left_foot_task = solver.add_frame_task("left_foot_tip", T_world_left)
left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

right_foot_task = solver.add_frame_task("right_foot_tip", T_world_right)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)

# Look at ball
look_at_ball = solver.add_axisalign_task("camera", np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 0.0]))
look_at_ball.configure("look_ball", "soft", 1.0)

# Trunk position
T_world_frame = robot.get_T_world_frame("trunk")
T_world_frame[2, 3] -= 0.06
init_trunk_z = T_world_frame[2, 3]

trunk_task = solver.add_frame_task("trunk", T_world_frame)
trunk_task.configure("trunk_task", "soft", 1.0, 1.0)

solver.add_regularization_task(1e-6)

# Setting custom target values for elbows
joints_task = solver.add_joints_task()
joints_task.configure("joints", "soft", 1.0)

joints_task.set_joints({"left_elbow": -2.0, "right_elbow": -2.0})

t = 0
dt = 0.005
start_t = time.time()
robot.update_kinematics()

for step in range(int(1e9)):
    if step % 10 == 0:
        viz.display(robot.state.q)

    w = 1
    ball = np.array([0.75 + np.cos(t * w) * 0.25, np.sin(t * w) * 0.7, 0.0])
    point_viz("ball", ball)

    if True:
        t0 = time.time()

        # Updating camera task
        camera_pos = robot.get_T_world_frame("camera")[:3, 3]
        look_at_ball.targetAxis_world = ball - camera_pos

        trunk_task.orientation().R_world_frame = tf.rotation([0, 0, 1], np.sin(t * 1.2))[:3, :3]
        # target = trunk_task.target_world
        # target[2] = init_trunk_z + np.sin(t)*.15
        # trunk_task.target_world = target

        qd = solver.solve(True)
        elapsed = time.time() - t0

        print(f"Computation time: {elapsed*1e6}µs")
        robot.update_kinematics()
        solver.dump_status()

    # Show some frames
    robot_frame_viz(robot, "camera")
    robot_frame_viz(robot, "left_foot_tip")
    robot_frame_viz(robot, "right_foot_tip")
    robot_frame_viz(robot, "trunk")

    # Show the CoM
    point_viz("com", robot.com_world(), radius=0.025, color=0xAAAAAA)

    # Spin-lock until the next tick
    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
