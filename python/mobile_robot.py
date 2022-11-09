import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
import tf
from visualization import robot_viz, robot_frame_viz, footsteps_viz

robot = placo.MobileRobot("sigmaban/")
robot.set_T_world_frame("left_foot_tip", placo.frame(np.eye(4)))

viewer = meshcat.Visualizer()
print(f"See at: {viewer.url()}")

viz = robot_viz(robot, viewer)

t = 0
dt = 0.05
while True:
    # robot.set_joint("left_knee", np.sin(t))
    robot.update_kinematics()

    viz.display(robot.state.q)
    robot_frame_viz(viewer, robot, "left_foot_tip")
    robot_frame_viz(viewer, robot, "right_foot_tip")
    robot_frame_viz(viewer, robot, "camera")

    target_x = np.cos(t)*2
    target_y = np.sin(t)*2
    planner = placo.FootstepsPlannerNaive(
    "left",
        placo.frame(tf.translation([0, 0, 0])),
        placo.frame(tf.translation([0, -0.1, 0])),
        0.1,
    )
    footsteps = planner.plan(
        placo.frame(tf.translation([target_x, target_y+ 0, 0])),
        placo.frame(tf.translation([target_x, target_y+ -0.1, 0])),
    )
    footsteps_viz(viewer, footsteps)

    t += dt
    time.sleep(dt)
