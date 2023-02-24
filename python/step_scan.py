import time
import placo
import pinocchio as pin
import numpy as np
import tf
from visualization import robot_viz, frame_viz, point_viz, robot_frame_viz

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")

robot.set_T_world_frame("left_foot", np.eye(4))
robot.update_kinematics()

solver = robot.make_solver()

scan_targets = [
    [x, y]
    for x in np.linspace(-0.1, 0.1, 100)
    for y in np.linspace(-0.1, 0.1, 100)
]
scan_target = 0

# T_world_trunk = np.eye(4)
# trunk_task = solver.add_frame_task("trunk", T_world_trunk)
# trunk_task.configure("trunk_task", "soft", 1.0, 1.0)

com_task = solver.add_com_task(np.array([0., 0., 0.32]))
com_task.configure("com_task", "soft", 1.0)

trunk_orientation_task = solver.add_orientation_task("trunk", np.eye(3))
trunk_orientation_task.configure("trunk_task", "soft", 1.0)

T_world_left = tf.translation((0., 0.055, 0.))
left_foot_task = solver.add_frame_task("left_foot", T_world_left)
left_foot_task.configure("left_foot_task", "soft", 1.0, 1.0)

T_world_right = tf.translation((0., -0.055, 0.))
right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot_task", "soft", 1.0, 1.0)

joints_task = solver.add_joints_task()
joints_task.configure("joints", "soft", 1.0)
joints_task.set_joints({
    "head_yaw": 0.,
    "head_pitch": 0.,
    "left_shoulder_pitch": 0.,
    "left_shoulder_roll": 0.,
    "left_elbow": -2.,
    "right_shoulder_pitch": 0.,
    "right_shoulder_roll": 0.,
    "right_elbow": -2.,
    
})

solver.add_regularization_task(1e-6)

viz = robot_viz(robot)
t = 0
dt = 0.01
start_t = time.time()
robot.update_kinematics()

scan_t = 0

for step in range(int(1e9)):
    w = 1

    scan_t += dt
    if scan_t > .01:
        scan_t = 0
        scan_target += 1
        cols = robot.self_collisions(False)
        if len(cols):
            print(cols[0].get_contact(0))
            point_viz("col", cols[0].get_contact(0), 0.025, 0xff0000)
        else:
            point_viz("col", [1e8]*3)

    if True:
        t0 = time.time()

        # T_world_trunk[2, 3] = 0.25 + np.sin(t)*0.05
        # trunk_task.T_world_frame = T_world_trunk

        # com_task.target_world = np.array([0., 0., 0.3 + np.sin(t)*0.03])

        # joints_task.set_joints({
        #     "left_shoulder_pitch":  np.sin(t*2.5)*0.5,
        #     "right_shoulder_pitch": np.sin(t*2.5)*0.5,
        # })

        T_world_right[:2, 3] = scan_targets[scan_target]
        right_foot_task.T_world_frame = T_world_right

        elapsed = time.time() - t0

        qd = solver.solve(True)
        # print(f"Computation time: {elapsed*1e6}µs")
        robot.update_kinematics()
        # solver.dump_status()

    # Show some frames
    if step % 3 == 0:
        viz.display(robot.state.q)
        robot_frame_viz(robot, "camera")
        robot_frame_viz(robot, "left_foot")
        robot_frame_viz(robot, "right_foot")
        robot_frame_viz(robot, "trunk")
        point_viz("com", robot.com_world(), radius=0.025, color=0xaaaaff)

    # Spin-lock until the next tick
    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
