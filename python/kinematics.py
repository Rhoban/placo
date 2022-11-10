import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
import tf
from visualization import robot_viz, robot_frame_viz, footsteps_viz, frame_viz

robot = placo.MobileRobot("sigmaban/")

robot.set_joint("left_knee", .1)
robot.set_joint("right_knee", .1)
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
viewer = meshcat.Visualizer()
print(f"See at: {viewer.url()}")
viz = robot_viz(robot, viewer)

t = 0
dt = 0.02
while True:
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    robot.update_kinematics()
    viz.display(robot.state.q)

    if True:
        # T_world_left[0, 3] = np.sin(t*3)*.1
        # T_world_right[0, 3] = -np.sin(t*3)*.1
        solver.add_frame_task("left_foot_tip", placo.frame(T_world_left), "hard", 1., 1.)
        solver.add_frame_task("right_foot_tip", placo.frame(T_world_right), "hard", 1., 1.)

        T_world_targetTrunk = tf.frame(xyz=[0, -0.05 + np.sin(3*t)*.1, 0.25])
        frame_viz(viewer, "trunk_target", T_world_targetTrunk, .25)
        
        solver.add_frame_task("trunk", placo.frame(T_world_targetTrunk), "soft", 1., 1.)
        solver.add_regularization_task(1e-3)

        qd = solver.solve(True)
        print(qd)

    # robot_frame_viz(viewer, robot, "left_foot_tip")
    # robot_frame_viz(viewer, robot, "right_foot_tip")
    # robot_frame_viz(viewer, robot, "camera")
    
    t += dt
    time.sleep(dt)
