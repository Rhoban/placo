import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
import tf
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz

# TODO: Update joint limits in URDF
# TODO: Regularize with reference posture
# TODO: Return tasks allowing retrieving of error after solve
# TODO: Move the viewer in visualization.py, avoid passing it everytime as argument to methods

robot = placo.MobileRobot("sigmaban/")

robot.set_joint("left_knee", 0.1)
robot.set_joint("right_knee", 0.1)
robot.update_kinematics()
robot.set_T_world_frame("left_foot_tip", placo.frame(np.eye(4)))
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)
# solver.mask_dof("left_shoulder_roll")
# solver.mask_dof("right_shoulder_roll")
# solver.mask_dof("left_shoulder_pitch")
# solver.mask_dof("right_shoulder_pitch")
# solver.mask_dof("left_elbow")
# solver.mask_dof("right_elbow")
# solver.mask_dof("head_pitch")
# solver.mask_dof("head_yaw")

# Retrieving initial position of the feet, com and trunk orientation
T_world_left = robot.get_T_world_frame("left_foot_tip").mat
T_world_right = robot.get_T_world_frame("right_foot_tip").mat
com_world = robot.com_world().copy()
R_world_trunk = robot.get_T_world_frame("trunk").mat[:3, :3]

# Creating the viewer
viewer = meshcat.Visualizer()
print(f"See at: {viewer.url()}")
viz = robot_viz(robot, viewer)

# left_foot_task = solver.add_frame_task("left_foot_tip", placo.frame(T_world_left))
# left_foot_task.configure("left_foot", "hard", 1.0, 1.0)

# right_foot_task = solver.add_frame_task("right_foot_tip", placo.frame(T_world_right))
# right_foot_task.configure("right_foot", "hard", 1.0, 1.0)

look_at_ball = solver.add_axisalign_task("camera", np.array([0., 0., 1.]), np.array([0., 0., 0.]))
look_at_ball.configure("look_ball", "soft", 1.)

T_world_frame = robot.get_T_world_frame("trunk").mat
T_world_frame[0, 3] += 0.5
T_world_frame[2, 3] += 0.2
T_world_frame[:3, :3] = tf.rotation([0, 1, 0], 1.1)[:3, :3]
trunk_task = solver.add_frame_task("trunk", placo.frame(T_world_frame))
trunk_task.configure("trunk", "soft", 1., 1.)

solver.add_regularization_task(1e-12)

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
    ball = np.array([0.5 + np.cos(t*w) * 0.25, np.sin(t*w) * 0.7, 0.0])
    point_viz(viewer, "ball", ball)

    if True:
        t0 = time.time()

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
        # robot.update_kinematics()
        elapsed = time.time() - t0

        print(f"Computation time: {elapsed*1e6}µs")
        robot.update_kinematics()
        solver.dump_status()

    robot_frame_viz(viewer, robot, "camera")
    robot_frame_viz(viewer, robot, "left_foot_tip")
    robot_frame_viz(viewer, robot, "right_foot_tip")
    robot_frame_viz(viewer, robot, "trunk")
    point_viz(viewer, "com", robot.com_world())

    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
