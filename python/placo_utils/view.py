import argparse
import placo
import time
from placo_utils.visualization import *

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("path", help="Path to the URDF")
arg_parser.add_argument("--frames", help="Frame to display", nargs="+")
args = arg_parser.parse_args()

robot = placo.RobotWrapper(args.path, placo.Flags.ignore_collisions)
viz = robot_viz(robot)

while True:
    viz.display(robot.state.q)
    for frame in args.frames:
        robot_frame_viz(robot, frame)

    time.sleep(1)
