import meshcat
import pinocchio as pin
import numpy as np
import meshcat.geometry as g
import meshcat.transformations as tf
import placo


def robot_viz(
    robot: placo.MobileRobot, viewer: meshcat.Visualizer
) -> pin.visualize.MeshcatVisualizer:
    """
    Builds an instance of pinocchio MeshcatVisualizer, which allows to push the model to the meshcat
    visualizer passed as parameter

    The robot can further be displayed using:

    > viz.display(q)
    """
    viz = pin.visualize.MeshcatVisualizer(
        robot.model, robot.collision_model, robot.visual_model
    )
    viz.initViewer(viewer=viewer)
    viz.loadViewerModel()

    return viz


cylinders: dict = {}


def frame_viz(
    vis: meshcat.Visualizer, name: str, T: np.ndarray, opacity: float = 1.0
) -> None:
    """
    Visualizes a given frame
    """
    global cylinders
    axises = {
        "x": (0xFF0000, (-np.pi / 2, [0, 0, 1])),
        "y": (0x00FF00, (0, [0, 1, 0])),
        "z": (0x0000FF, (np.pi / 2, [1, 0, 0])),
    }

    for axis_name in axises:
        color, rotate = axises[axis_name]
        node_name = f"frames/{name}/{axis_name}"

        if node_name not in cylinders:
            cylinders[node_name] = vis[node_name]
            cylinders[node_name].set_object(
                g.Cylinder(0.1, 0.005),
                g.MeshLambertMaterial(color=color, opacity=opacity),
            )
        obj = cylinders[node_name]

        obj.set_transform(
            T @ tf.rotation_matrix(*rotate) @ tf.translation_matrix([0, 0.05, 0])
        )


def robot_frame_viz(
    vis: meshcat.Visualizer, robot: placo.MobileRobot, frame: str
) -> None:
    """
    Draw a frame from the robot
    """
    frame_viz(vis, frame, robot.get_T_world_frame(frame).mat)


steps: int = 0


def footsteps_viz(vis: meshcat.Visualizer, footsteps: placo.Footsteps) -> None:
    global steps
    if len(footsteps) < steps:
        vis["footsteps"].delete()
    steps = len(footsteps)

    k = 0
    for footstep in footsteps:
        k += 1
        polygon = [[*xy, 0] for xy in footstep.support_polygon()]
        polygon = np.array([*polygon, polygon[-1]])
        color = 0xFF3333 if str(footstep.side) == "left" else 0x33FF33

        vis["footsteps"][str(k)].set_object(
            g.LineLoop(g.PointsGeometry(polygon.T), g.MeshBasicMaterial(color=color))
        )
