import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
from placo_utils.visualization import robot_viz, frame_viz, point_viz, robot_frame_viz
from placo_utils.tf import tf


def generate_target():
    r = pin.exp6(np.array([0.0, 0.0, 0.0, *np.random.uniform([-1.0] * 3, [1.0] * 3)]))
    t = pin.exp6(np.array([*np.random.uniform([1.0, -1.0, 0.5], [1.5, 1.0, 1.2]), 0.0, 0.0, 0.0]))
    return t * r


# Loading the robot
robot = placo.RobotWrapper("6axis/", placo.Flags.ignore_collisions)

robot.update_kinematics()

solver = robot.make_solver()

# Robot is fixed
solver.mask_fbase(True)

# Creating the viewer
viz = robot_viz(robot)

# Keep left and right foot on the floor
T_world_effector = tf.translation_matrix((1.0, 0.0, 1.0))
effector_task = solver.add_frame_task("effector", T_world_effector)
effector_task.configure("effector", "soft", 1.0, 1.0)

solver.add_regularization_task(1e-6)

# Setting custom target values for elbows
joints_task = solver.add_joints_task()

joints_task.set_joints(
    {
        "r1": 0.0,
        "r2": 0.0,
        "r3": 0.0,
        "r4": 0.0,
        "r5": 0.0,
        "r6": 0.0,
    }
)
joints_task.configure("joints", "soft", 1e-4)

t = 0
dt = 0.005
start_t = time.time()
last_frame = 0
robot.update_kinematics()
solver.configure_limits(True, True, False)
last_target = -1e3

elapseds = []
for step in range(int(1e9)):
    if time.time() - last_frame > 0.05:
        last_frame = time.time()
        viz.display(robot.state.q)

    w = 1

    if True:
        t0 = time.time()

        if time.time() - last_target > 2.5:
            T_world_effector = np.array(generate_target())
            last_target = time.time()

        effector_task.T_world_frame = T_world_effector

        qd = solver.solve(True)
        elapsed = time.time() - t0

        elapseds.append(elapsed)
        print(f"Computation time: {np.mean(elapseds)*1e6}µs")
        robot.update_kinematics()
        solver.dump_status()

    # Show some frames
    robot_frame_viz(robot, "effector")
    frame_viz("effector_target", T_world_effector)

    # Spin-lock until the next tick
    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
