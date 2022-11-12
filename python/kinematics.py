import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
import tf
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz

# TODO: Clean this to make it a proper simple example
# TODO: Do something about conversions from numpy 4x4 matrices to Affine3d?
# TODO: Update joint limits in URDF

robot = placo.MobileRobot("sigmaban/")

robot.set_joint("left_knee", 0.1)
robot.set_joint("right_knee", 0.1)
robot.update_kinematics()
robot.set_T_world_frame("left_foot_tip", placo.frame(np.eye(4)))
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)

# Retrieving initial position of the feet, com and trunk orientation
T_world_left = robot.get_T_world_frame("left_foot_tip").mat
T_world_right = robot.get_T_world_frame("right_foot_tip").mat
com_world = robot.com_world().copy()
R_world_trunk = robot.get_T_world_frame("trunk").mat[:3, :3]

# Creating the viewer
viz = robot_viz(robot)

left_foot_task = solver.add_pose_task("left_foot_tip", placo.frame(T_world_left))
left_foot_task.configure("left_foot", "soft", 1.0)

right_foot_task = solver.add_relative_frame_task(
    "left_foot_tip", "right_foot_tip", placo.frame(tf.translation([0, -0.1, 0]))
)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)

# right_foot_orn_task = solver.add_orientation_task("right_foot_tip", np.eye(3))
# right_foot_orn_task.configure("right_foot_orn", "soft", 1.0)

look_at_ball = solver.add_axisalign_task("camera", np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 0.0]))
look_at_ball.configure("look_ball", "soft", 1.0)

T_world_frame = robot.get_T_world_frame("trunk").mat
T_world_frame[2, 3] -= 0.06
# trunk_task = solver.add_frame_task("trunk", placo.frame(T_world_frame))
# trunk_task.configure("trunk", "soft", 1., 1.)
trunk_task = solver.add_position_task("trunk", T_world_frame[:3, 3])
trunk_task.configure("trunk_task", "soft", 1.0)
trunk_orientation_task = solver.add_orientation_task("trunk", np.eye(3))
trunk_orientation_task.configure("trunk_orn", "soft", 1.0)

solver.add_regularization_task(1e-6)

# Elbows
joints_task = solver.add_joints_task()
joints_task.configure("joints", "soft", 1.0)

joints_task.set_joints({"left_elbow": -2.0, "right_elbow": -2.0})

t = 0
dt = 0.005
K = 0
start_t = time.time()
robot.update_kinematics()

while True:
    K += 1
    if K % 10 == 0:
        viz.display(robot.state.q)

    w = 1
    ball = np.array([0.5 + np.cos(t * w) * 0.25, np.sin(t * w) * 0.7, 0.0])
    point_viz("ball", ball)

    if True:
        t0 = time.time()

        # Moving left foot
        T_world_left[:2, 3] = [0 + np.cos(t) * 0.05, np.sin(t) * 0.05]
        # T_world_left[2, 3] = 0.02 + np.sin(t)*0.01
        # T_world_left[:3, :3] = tf.rotation([0, 0, 1], np.sin(t * 3) * 0.2)[:3, :3]
        # T_world_left[:3, :3] = T_world_left[:3, :3] @ tf.rotation([0, 1, 0], np.sin(t * 2) * 0.2)[:3, :3]

        left_foot_task.T_world_frame = placo.frame(T_world_left)

        right_foot_task.T_a_b = placo.frame(tf.translation([0, -0.1, 0]) @ tf.rotation([0, 0, 1], np.pi / 2))

        # right_foot_orn_task.R_world_target = tf.rotation([0, 0, 1], np.sin(t * 3) * 0.3)[:3, :3]

        # right_foot_task.position().target_world = np.array([0., -0.1, 0.1 + np.sin(t)*0.05])
        # right_foot_task.orientation().R_world_target = tf.rotation([0, 0, 1], np.sin(t*1.1)*.2)[:3, :3]

        # R = (tf.rotation([0, 0, 1], t/2) @ tf.rotation([0, 1, 0], 1.1))[:3, :3]
        # trunk_task.orientation().R_world_target = T_world_frame[:3, :3] = R
        camera_pos = robot.get_T_world_frame("camera").mat[:3, 3]
        look_at_ball.targetAxis_world = ball - camera_pos

        # # Controlling the com
        # T_world_targetTrunk = tf.frame(xyz=[0, -0.05 + np.sin(t) * 0.1, 0.35 + np.sin(t * 1.1) * 0.01])
        # frame_viz(viewer, "trunk_target", T_world_targetTrunk, 0.25)
        # solver.add_com_task(placo.frame(T_world_targetTrunk).mat[:3, 3], "soft", 1.0)
        # solver.add_orientation_task("trunk", np.eye(3), "soft", 1.0)
        # solver.add_regularization_task(1e-5)

        # # Looking at the ball
        # solver.add_axisalign_task("camera", np.array([0, 0, 1]), target_vector, "soft", 1e-4)

        # # Setting the arms target
        # solver.add_joint_task("left_shoulder_roll", 0.0, "soft", 1.0)
        # solver.add_joint_task("left_shoulder_pitch", 0.5, "soft", 1.0)
        # solver.add_joint_task("left_elbow", -1.5, "soft", 1.0)

        # solver.add_joint_task("right_shoulder_roll", 0.0, "soft", 1.0)
        # solver.add_joint_task("right_shoulder_pitch", 0.5, "soft", 1.0)
        # solver.add_joint_task("right_elbow", -1.5, "soft", 1.0)

        # for k in range(1000):
        qd = solver.solve(True)
        robot.update_kinematics()
        # robot.update_kinematics()
        elapsed = time.time() - t0

        print(f"Computation time: {elapsed*1e6}µs")
        robot.update_kinematics()
        solver.dump_status()
        # exit()

    robot_frame_viz(robot, "camera")
    robot_frame_viz(robot, "left_foot_tip")
    robot_frame_viz(robot, "right_foot_tip")
    robot_frame_viz(robot, "trunk")
    point_viz("com", robot.com_world(), radius=0.025, color=0xAAAAAA)

    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
