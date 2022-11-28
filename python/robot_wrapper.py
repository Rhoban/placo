import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
import tf
from visualization import robot_viz, robot_frame_viz, footsteps_viz, frame_viz

robot = placo.RobotWrapper("6axis/")

solver = robot.make_solver()
solver.mask_fbase(True)
solver.noise = 0.

task = solver.add_frame_task("effector", tf.frame([0., 0., 1.], 1., [1.0, 0., 1.0]))
task.configure("effector", "soft", 1e-2, 1.)

# rtask = solver.add_frame_task("effector", tf.frame([0., 0., 1.], 1., [1.0, 0., 1.0]))
# rtask.configure("effector", "soft", 1e-1, 1e-1)

# task = solver.add_pose_task("effector", tf.frame([0., 0., 1.], 1., [1.0, 0., 1.0]))

solver.add_regularization_task(1e-6)

solver.configure_limits(False, True, False)
solver.dt = 1e-2

viz = robot_viz(robot)

def generate_target():
    r = pin.exp6(np.array([0.0, 0.0, 0.0, *np.random.uniform([-1.0] * 3, [1.0] * 3)]))
    t = pin.exp6(np.array([*np.random.uniform([1.0, -1.0, 0.5], [1.5, 1.0, 1.2]), 0.0, 0.0, 0.0]))
    return (t * r).np
T = generate_target()

t = 0
dt = 0.01
while True:
    robot.update_kinematics()
    solver.solve(True)

    T_world_effector = robot.get_T_world_frame("effector")
    # rtask.T_world_frame = T_world_effector

    error = 0
    # error += task.error()
    error += task.position().error()
    error += task.orientation().error()

    if error < 1e-2:
        T = generate_target()

    task.T_world_frame = T

    viz.display(robot.state.q)
    robot_frame_viz(robot, "effector")

    frame_viz("target", T, .25)    

    t += dt
    time.sleep(dt)
