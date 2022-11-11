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

t = 0
dt = 0.005
K = 0
start_t = time.time()
while True:
    robot.update_kinematics()
    K += 1
    if K % 10 == 0:
        viz.display(robot.state.q)

    ball = np.array([0.5 + np.cos(t) * 0.25, np.sin(t) * 0.7, 0.0])
    point_viz(viewer, "ball", ball)

    if True:
        t0 = time.time()

        T_world_right[2, 3] = .02 + np.sin(t*2)*0.02

        solver.clear_tasks()

        solver.add_frame_task("left_foot_tip", placo.frame(T_world_left)).configure("left_foot", "hard", 1., 1.)
        solver.add_frame_task("right_foot_tip", placo.frame(T_world_right)).configure("right_foot", "hard", 1., 1.)

        # # Controlling the com
        # T_world_targetTrunk = tf.frame(xyz=[0, -0.05 + np.sin(t) * 0.1, 0.35 + np.sin(t * 1.1) * 0.01])
        # frame_viz(viewer, "trunk_target", T_world_targetTrunk, 0.25)
        # solver.add_com_task(placo.frame(T_world_targetTrunk).mat[:3, 3], "soft", 1.0)
        # solver.add_orientation_task("trunk", np.eye(3), "soft", 1.0)
        # solver.add_regularization_task(1e-5)

        # # Looking at the ball
        # camera_pos = robot.get_T_world_frame("camera").mat[:3, 3]
        # target_vector = ball - camera_pos
        # solver.add_axisalign_task("camera", np.array([0, 0, 1]), target_vector, "soft", 1e-4)

        # # Setting the arms target
        # solver.add_joint_task("left_shoulder_roll", 0.0, "soft", 1.0)
        # solver.add_joint_task("left_shoulder_pitch", 0.5, "soft", 1.0)
        # solver.add_joint_task("left_elbow", -1.5, "soft", 1.0)

        # solver.add_joint_task("right_shoulder_roll", 0.0, "soft", 1.0)
        # solver.add_joint_task("right_shoulder_pitch", 0.5, "soft", 1.0)
        # solver.add_joint_task("right_elbow", -1.5, "soft", 1.0)

        solver.add_regularization_task(1e-6)

        qd = solver.solve(True)
        elapsed = time.time() - t0

    robot_frame_viz(viewer, robot, "camera")
    robot_frame_viz(viewer, robot, "left_foot_tip")
    robot_frame_viz(viewer, robot, "right_foot_tip")
    robot_frame_viz(viewer, robot, "trunk")
    point_viz(viewer, "com", robot.com_world())

    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
