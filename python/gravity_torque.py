import time
import meshcat
import placo
import pinocchio as pin
import numpy as np
from placo_utils.visualization import robot_viz, frame_viz, point_viz, robot_frame_viz
from placo_utils.tf import tf

# Loading the robot
robot = placo.HumanoidRobot("sigmaban/")

# robot.set_joint("left_knee", 0.1)
# robot.set_joint("right_knee", 0.1)
# robot.update_kinematics()
robot.set_T_world_frame("left_foot", np.eye(4))
robot.update_kinematics()

solver = robot.make_solver()

# Retrieving initial position of the feet, com and trunk orientation
T_world_left = robot.get_T_world_frame("left_foot")
T_world_right = robot.get_T_world_frame("right_foot")

# Creating the viewer
viz = robot_viz(robot)

# Keep left and right foot on the floor
left_foot_task = solver.add_frame_task("left_foot", T_world_left)
left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)
right_foot_task.T_world_frame = T_world_right

# Trunk position
T_world_frame = robot.get_T_world_frame("trunk")
T_world_frame[2, 3] -= 0.08
init_trunk_pos = T_world_frame[:3, 3]

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

elapseds = []
for step in range(int(1e9)):
    if step % 10 == 0:
        viz.display(robot.state.q)

    w = 1

    left_result = placo.GravityTorques.compute_gravity_torques(robot, ["left_foot"], 0.08, 0.06, 0.1)
    right_result = placo.GravityTorques.compute_gravity_torques(robot, ["right_foot"], 0.08, 0.06, 0.1)

    result = placo.GravityTorques.compute_gravity_torques(robot, ["left_foot", "right_foot"], 0.08, 0.06, 0.1)
    if result.success:
      if left_result.success:
          print(robot.static_gravity_compensation_torques("left_foot") - left_result.tau)

      print("~")
      print("Left hip roll tau: %f" % result.tau_dict(robot)["left_hip_roll"])
      print("F1: %s" % result.contact_wrenches[0][:3])
      print("M1: %s" % result.contact_wrenches[0][3:])
      print("F2: %s" % result.contact_wrenches[1][:3])
      print("M2: %s" % result.contact_wrenches[1][3:])

      left_fz = result.contact_wrenches[0, 2]
      left_mx = result.contact_wrenches[0, 3]
      left_my = result.contact_wrenches[0, 4]
      zmp_left = [left_my / left_fz, left_mx / left_fz, 0]
      zmp_left = (robot.get_T_world_frame("left_foot") @ [*zmp_left, 1])[:3]
      point_viz("zmp_left", zmp_left, left_fz*1e-3, 0xAAFFAA if left_result.success else 0xFF00AA)

      right_fz = result.contact_wrenches[1, 2]
      right_mx = result.contact_wrenches[1, 3]
      right_my = result.contact_wrenches[1, 4]
      zmp_right = [right_my / right_fz, right_mx / right_fz, 0]
      zmp_right = (robot.get_T_world_frame("right_foot") @ [*zmp_right, 1])[:3]
      point_viz("zmp_right", zmp_right, right_fz*1e-3, 0xAAFFAA if right_result.success else 0xFF00AA)
    else:
        print("(unfeasible contacts)")

    if True:
        t0 = time.time()

        target_trunk = init_trunk_pos.copy()
        target_trunk[1] += np.sin(t)*0.07
        trunk_task.position().target_world = target_trunk

        qd = solver.solve(True)
        elapsed = time.time() - t0

        robot.update_kinematics()

    # Show some frames
    robot_frame_viz(robot, "camera")
    robot_frame_viz(robot, "left_foot")
    robot_frame_viz(robot, "right_foot")
    robot_frame_viz(robot, "trunk")

    # Show the CoM
    point_viz("com", robot.com_world(), radius=0.025, color=0xAAAAAA)

    # Spin-lock until the next tick
    t += dt
    while time.time() < start_t + t:
        time.sleep(1e-3)
