import numpy as np
import time
import matplotlib.pyplot as plt
import placo
import tf
import numpy as np
import argparse


def plot_footsteps(footsteps: list):
    polygons = {}
    for footstep in footsteps:
        if isinstance(footstep, placo.Footstep):
            key = footstep.side
        else:
            if len(footstep.footsteps) == 1:
                key = footstep.footsteps[0].side
            else:
                key = "double"

        if key not in polygons:
            polygons[key] = []

        points = list(footstep.support_polygon())
        points.append(points[0])
        polygons[key] += points
        polygons[key].append([np.nan, np.nan])

    for entry in polygons:
        p = np.array(polygons[entry])
        plt.plot(p.T[0], p.T[1], label=entry)
    plt.legend()
    plt.axis("equal")
    plt.gca().set_adjustable("box")
    plt.grid()


def draw_footsteps(footsteps, animate=False, show=True):
    if animate:
        # Handling footsteps animation
        all_points = np.vstack([list(footstep.support_polygon()) for footstep in footsteps])
        x_min, x_max = np.min(all_points.T[0]) - 0.1, np.max(all_points.T[0]) + 0.1
        y_min, y_max = np.min(all_points.T[1]) - 0.1, np.max(all_points.T[1]) + 0.1
        for step in range(len(footsteps)):
            plt.clf()
            plot_footsteps(footsteps[: step + 1])
            plt.xlim(x_min, x_max)
            plt.ylim(y_min, y_max)
            plt.pause(0.25)
    else:
        plot_footsteps(footsteps)

    if show:
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process some integers.")
    parser.add_argument("-a", "--animate", action="store_true", help="Animate the plotting")
    parser.add_argument("-x", type=float, default=1.0, help="Target x")
    parser.add_argument("-y", type=float, default=0, help="Target y")
    parser.add_argument("-t", "--theta", type=float, default=0, help="Target yaw (theta)")
    parser.add_argument("-f", "--feet_spacing", type=float, default=0.2, help="Feet spacing")
    parser.add_argument("-w", "--foot_width", type=float, default=0.1, help="Foot width")
    parser.add_argument("-l", "--foot_length", type=float, default=0.15, help="Foot length")
    parser.add_argument("-ds", "--double_supports_start", action="store_true", help="Adds double support at begining")
    parser.add_argument("-dm", "--double_supports_middle", action="store_true", help="Adds double support at each step")
    parser.add_argument("-de", "--double_supports_end", action="store_true", help="Adds double support at end")
    args = parser.parse_args()

    # Creating initial and target
    feet_spacing = args.feet_spacing

    T_center_left = tf.translation([0.0, feet_spacing / 2, 0.0])
    T_center_right = tf.translation([0.0, -feet_spacing / 2, 0.0])

    T_world_center = tf.frame([0, 0, 1], args.theta, [args.x, args.y, 0])
    T_world_targetLeft = T_world_center @ T_center_left
    T_world_targetRight = T_world_center @ T_center_right

    start = time.time()

    # Naive planner
    # planner = placo.FootstepsPlannerNaive("left", placo.frame(T_center_left), placo.frame(T_center_right))
    # planner.parameters.feet_spacing = args.feet_spacing
    # planner.parameters.foot_width = args.foot_width
    # planner.parameters.foot_length = args.foot_length
    # footsteps = planner.plan(placo.frame(T_world_targetLeft), placo.frame(T_world_targetRight))

    # Repetitive planner
    planner = placo.FootstepsPlannerRepetitive("left", placo.frame(T_center_left), placo.frame(T_center_right))
    planner.parameters.feet_spacing = args.feet_spacing
    planner.parameters.foot_width = args.foot_width
    planner.parameters.foot_length = args.foot_length
    footsteps = planner.plan(0.1, 0., 0.314, 10)

    # If using double support, transform the result in supports instead of footsteps
    if args.double_supports_start or args.double_supports_middle or args.double_supports_end:
        footsteps = planner.make_double_supports(
            footsteps, args.double_supports_start, args.double_supports_middle, args.double_supports_end
        )

    elapsed = time.time() - start

    print(f"{len(footsteps)} steps, computation time: {elapsed*1e6}µs.")

    draw_footsteps(footsteps, args.animate)
