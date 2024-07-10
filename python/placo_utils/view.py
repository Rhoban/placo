import argparse
import placo
import time
from placo_utils.visualization import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("path", help="Path to the URDF")
arg_parser.add_argument("--frames", help="Frame to display", nargs="+")
arg_parser.add_argument("--animate", help="Animate the robot", action="store_true")
args = arg_parser.parse_args()

robot = placo.RobotWrapper(args.path, placo.Flags.ignore_collisions)
robot.update_kinematics()

print("Joint names:")
print(list(robot.joint_names()))

print("Frame names:")
print(list(robot.frame_names()))

viz = robot_viz(robot)
t = 0

while True:
    viz.display(robot.state.q)

    if args.frames:
        for frame in args.frames:
            robot_frame_viz(robot, frame)

    if args.animate:
        for joint in robot.joint_names():
            robot.set_joint(joint, np.sin(t))
            robot.update_kinematics()

    t += 0.01
    time.sleep(0.01)
