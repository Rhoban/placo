import argparse
import placo
import time
from placo_utils.visualization import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("path", help="Path to the URDF")
arg_parser.add_argument("--frames", help="Frames to show (all the frames are in the viewer, hidden by default)", nargs="+")
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

# Frames (each name once). They are all added to the viewer, where they can be shown or hidden from the scene tree:
# hidden by default, except the ones given with --frames
frames = list(dict.fromkeys(robot.frame_names()))
for frame in frames:
    robot_frame_viz(robot, frame)
    nodes[f"frames/{robot_names[robot]}_{frame}"][1].visible = bool(args.frames and frame in args.frames)

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
