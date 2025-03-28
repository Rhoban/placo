import meshcat
import pinocchio as pin
import pinocchio.visualize as pin_viz
import numpy as np
import meshcat.geometry as g
import meshcat.transformations as tf
import placo


viewer = None
robot_names: dict = {}


def get_viewer() -> meshcat.Visualizer:
    """
    Gets the meshcat viewer, if it doesn't exist, create it
    """
    global viewer

    if viewer is None:
        viewer = meshcat.Visualizer()

        print(f"Viewer URL: {viewer.url()}")

    return viewer


def robot_viz(
    robot: placo.RobotWrapper, name: str = "robot"
) -> pin_viz.MeshcatVisualizer:
    """
    Builds an instance of pinocchio MeshcatVisualizer, which allows to push the model to the meshcat
    visualizer passed as parameter

    The robot can further be displayed using:

    > viz.display(q)
    """
    global robot_names

    robot_names[robot] = name
    viz = pin_viz.MeshcatVisualizer(
        robot.model, robot.collision_model, robot.visual_model
    )
    viz.initViewer(viewer=get_viewer())
    viz.loadViewerModel(name)

    return viz


cylinders: dict = {}


def frame_viz(name: str, T: np.ndarray, opacity: float = 1.0, scale: float = 1.0) -> None:
    """
    Visualizes a given frame
    """
    global cylinders
    vis = get_viewer()
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
            T @ tf.scale_matrix(scale) @ tf.rotation_matrix(*rotate) @ tf.translation_matrix([0, 0.05, 0])
        )


def point_viz(
    name: str,
    point: np.ndarray,
    radius: float = 0.01,
    color: float = 0xFF0000,
    opacity: float = 1.0,
) -> None:
    """
    Prints a point (sphere)
    """
    vis = get_viewer()
    vis["point"][name].set_object(
        g.Sphere(radius), g.MeshPhongMaterial(color=color, opacity=opacity)
    )
    vis["point"][name].set_transform(tf.translation_matrix(point))


points_sizes = {}


def points_viz(
    name: str,
    points: np.ndarray,
    radius: float = 0.01,
    color: float = 0xFF0000,
    opacity: float = 1.0,
) -> None:
    """
    Prints a point (sphere)
    """
    global points_sizes
    vis = get_viewer()
    k = 0
    for point in points:
        entry = f"{name}_{k}"
        k += 1
        vis["points"][name][entry].set_object(
            g.Sphere(radius), g.MeshPhongMaterial(color=color, opacity=opacity)
        )
        vis["points"][name][entry].set_transform(tf.translation_matrix(point))
    while k < points_sizes.get(name, 0):
        vis["points"][name][f"{name}_{k}"].delete()
        k += 1
    points_sizes[name] = len(points)


def robot_frame_viz(robot: placo.RobotWrapper, frame: str, opacity: float = 1.0, scale: float = 1.0) -> None:
    """
    Draw a frame from the robot
    """
    node_name = f"{robot_names[robot]}_{frame}"
    frame_viz(node_name, robot.get_T_world_frame(frame), opacity, scale)


steps: int = 0


def footsteps_viz(footsteps: placo.Footsteps, T: np.ndarray = np.eye(4)) -> None:
    global steps
    vis = get_viewer()

    if len(footsteps) < steps:
        vis["footsteps"].delete()
    steps = len(footsteps)

    k = 0
    for footstep in footsteps:
        k += 1
        polygon = [(T @ [*xy, 0, 1])[:3] for xy in footstep.support_polygon()]
        polygon = np.array([*polygon, polygon[-1]])

        if isinstance(footstep, placo.Support):
            if len(footstep.footsteps) >= 2:
                color = 0x1111AA
            else:
                color = (
                    0xFF3333 if str(footstep.footsteps[0].side) == "left" else 0x33FF33
                )
        else:
            color = 0xFF3333 if str(footstep.side) == "left" else 0x33FF33

        vis["footsteps"][str(k)].set_object(
            g.LineLoop(g.PointsGeometry(polygon.T), g.MeshBasicMaterial(color=color))
        )


def line_viz(name: str, points: np.ndarray, color: float = 0xFF0000) -> None:
    """
    Prints a line
    """
    vis = get_viewer()
    vis["lines"][name].set_object(
        g.LineSegments(g.PointsGeometry(points.T), g.LineBasicMaterial(color=color))
    )


def arrow_viz(
    name: str,
    point_from: np.ndarray,
    point_to: np.ndarray,
    color: float = 0xFF0000,
    radius: float = 0.003,
) -> None:
    """
    Prints an arrow
    """
    head_length = radius * 3
    vis = get_viewer()
    length = np.linalg.norm(point_to - point_from)
    length = max(1e-3, length - head_length)

    cylinder = g.Cylinder(length, radius)
    head = g.Cylinder(head_length, 2 * radius, 0.0, 2 * radius)

    T = tf.translation_matrix(point_from)
    if np.linalg.norm(point_to - point_from) > 1e-6:
        T[:3, :3] = placo.rotation_from_axis(
            "y", (point_to - point_from) / np.linalg.norm(point_to - point_from)
        )

    T_cylinder = T @ tf.translation_matrix(np.array([0, length / 2.0, 0.0]))
    T_head = T @ tf.translation_matrix(np.array([0, length + head_length / 2.0, 0.0]))

    vis["arrows"][name]["cylinder"].set_object(
        cylinder, g.MeshBasicMaterial(color=color)
    )
    vis["arrows"][name]["cylinder"].set_transform(T_cylinder)
    vis["arrows"][name]["head"].set_object(head, g.MeshBasicMaterial(color=color))
    vis["arrows"][name]["head"].set_transform(T_head)


previous_contacts: int = 0


def contacts_viz(solver: placo.DynamicsSolver, ratio=0.1, radius=0.005):
    global previous_contacts
    robot = solver.robot
    frames = robot.frame_names()
    k = 0

    for _ in range(solver.count_contacts()):
        contact = solver.get_contact(k)
        if not contact.active:
            continue

        if isinstance(contact, placo.PointContact):
            frame_name = frames[contact.position_task().frame_index]
            T_world_frame = robot.get_T_world_frame(frame_name)
            arrow_viz(
                f"contact_{k}",
                T_world_frame[:3, 3],
                T_world_frame[:3, 3] + contact.wrench * ratio,
                color=0x00FF00,
                radius=radius,
            )
        elif isinstance(contact, placo.Contact6D):
            frame_name = frames[contact.position_task().frame_index]
            T_world_frame = robot.get_T_world_frame(frame_name)
            wrench = T_world_frame[:3, :3] @ contact.wrench[:3]

            if np.linalg.norm(wrench) < 1e-6:
                origin = T_world_frame[:3, 3]
            else:
                origin = T_world_frame[:3, 3] + T_world_frame[:3, :3] @ contact.zmp()

            arrow_viz(
                f"contact_{k}",
                origin,
                origin + wrench * ratio,
                color=0x00FFAA,
                radius=radius,
            )
        elif isinstance(contact, placo.LineContact):
            frame_name = frames[contact.position_task().frame_index]
            T_world_frame = robot.get_T_world_frame(frame_name)
            wrench = T_world_frame[:3, :3] @ contact.wrench[:3]

            if np.linalg.norm(wrench) < 1e-6:
                origin = T_world_frame[:3, 3]
            else:
                origin = T_world_frame[:3, 3] + T_world_frame[:3, :3] @ contact.zmp()

            arrow_viz(
                f"contact_{k}",
                origin,
                origin + wrench * ratio,
                color=0xFF33AA,
                radius=radius,
            )
        elif isinstance(contact, placo.ExternalWrenchContact):
            frame_name = frames[contact.frame_index]
            T_world_frame = robot.get_T_world_frame(frame_name)
            arrow_viz(
                f"contact_{k}",
                T_world_frame[:3, 3],
                T_world_frame[:3, 3] + contact.w_ext[:3] * ratio,
                color=0xFF2222,
                radius=radius,
            )

        k += 1

    vis = get_viewer()
    k_d = k
    while k_d < previous_contacts:
        vis["arrows"][f"contact_{k_d}"].delete()
        k_d += 1
    previous_contacts = k
