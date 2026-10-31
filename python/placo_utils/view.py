import argparse
import placo
import time
from placo_utils.visualization import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("path", help="Path to the URDF")
arg_parser.add_argument("--frames", help="Frames to display (all by default)", nargs="+")
arg_parser.add_argument("--mjcf", action="store_true", default=False)
arg_parser.add_argument("--animate", help="Animate the robot", action="store_true")
args = arg_parser.parse_args()

flags = placo.Flags.ignore_collisions
if args.mjcf:
    flags += placo.Flags.mjcf
robot = placo.RobotWrapper(args.path, flags)
robot.update_kinematics()

print("Joint names:")
print(list(robot.joint_names()))

print("Frame names:")
print(list(robot.frame_names()))

viz = robot_viz(robot)

# The joints are controlled with sliders in the viewer
sliders = joints_sliders(robot)

# Frames to display (they can be shown or hidden from the viewer scene tree), each name once
frames = args.frames if args.frames else list(dict.fromkeys(robot.frame_names()))

t = 0

while True:
    for joint, slider in sliders.items():
        if args.animate:
            robot.set_joint(joint, np.sin(t))
            slider.value = float(np.clip(robot.get_joint(joint), slider.min, slider.max))
        else:
            robot.set_joint(joint, slider.value)
    robot.update_kinematics()

    viz.display(robot.state.q)
    for frame in frames:
        robot_frame_viz(robot, frame)

    t += 0.01
    time.sleep(0.01)
