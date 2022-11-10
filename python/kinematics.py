import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
import tf
from visualization import robot_viz, robot_frame_viz, footsteps_viz, frame_viz

# TODO: Update joint limits in URDF
# TODO: Regularize with reference posture
# TODO: Return tasks allowing retrieving of error after solve

robot = placo.MobileRobot("sigmaban/")

robot.set_joint("left_knee", .1)
robot.set_joint("right_knee", .1)
robot.update_kinematics()
robot.set_T_world_frame("left_foot_tip", placo.frame(np.eye(4)))
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)
# solver.mask_dof("left_shoulder_roll")
# solver.mask_dof("right_shoulder_roll")
solver.mask_dof("left_shoulder_pitch")
solver.mask_dof("right_shoulder_pitch")
solver.mask_dof("left_elbow")
solver.mask_dof("right_elbow")
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

    if True:
        t0 = time.time()
        solver.add_frame_task("left_foot_tip", placo.frame(T_world_left), "hard", 1., 1.)
        solver.add_frame_task("right_foot_tip", placo.frame(T_world_right), "hard", 1., 1.)

        T_world_targetTrunk = tf.frame(xyz=[0, -0.05 + np.sin(t)*0.1, 0.36])
        frame_viz(viewer, "trunk_target", T_world_targetTrunk, .25)
        
        solver.add_com_task(placo.frame(T_world_targetTrunk).mat[:3, 3], "soft", 1.)
        # solver.add_orientation_task("trunk", np.eye(3), "soft", 1.)
        solver.add_regularization_task(1e-5)

        qd = solver.solve(True)
        elapsed = time.time() - t0

    # robot.set_joint("left_shoulder_pitch", -1 + np.sin(t)*1)
    # robot.set_joint("right_shoulder_pitch", -1 + np.sin(t)*1)

    # robot_frame_viz(viewer, robot, "left_foot_tip")
    # robot_frame_viz(viewer, robot, "right_foot_tip")
    # robot_frame_viz(viewer, robot, "camera")
    
    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
