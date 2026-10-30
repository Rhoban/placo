import viser
import trimesh
import numpy as np
from pinocchio.visualize import ViserVisualizer
import placo
from . import tf


viewer = None
robot_names: dict = {}


def get_viewer() -> viser.ViserServer:
    """
    Gets the viser server, if it doesn't exist, create it
    """
    global viewer

    if viewer is None:
        viewer = viser.ViserServer()

        # Closer than the default (3, 3, 3), which is far for most robots
        viewer.initial_camera.position = (1.5, 1.0, 1.0)
        viewer.initial_camera.look_at = (0.0, 0.0, 0.2)

    return viewer


def robot_viz(robot: placo.RobotWrapper, name: str = "robot") -> ViserVisualizer:
    """
    Builds an instance of pinocchio ViserVisualizer, which allows to push the model to the viser
    viewer

    The robot can further be displayed using:

    > viz.display(q)
    """
    global robot_names

    robot_names[robot] = name
    viz = ViserVisualizer(robot.model, robot.collision_model, robot.visual_model)
    viz.initViewer(viewer=get_viewer())
    viz.loadViewerModel(name)

    return viz


def rgb(color: int) -> tuple:
    """
    Converts a 0xRRGGBB color to a (r, g, b) tuple
    """
    color = int(color)
    return ((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF)


def set_pose(handle, T: np.ndarray) -> None:
    """
    Sets the pose of a viser scene node from a 4x4 transformation matrix
    """
    handle.position = T[:3, 3]
    handle.wxyz = tf.quaternion_from_matrix(T)


# Scene nodes, as (parameters, handle), reused as long as their parameters don't change
nodes: dict = {}


def node(name: str, parameters: tuple, create):
    """
    Gets the node with the given name, created by create() if it doesn't exist or if it was created with other
    parameters
    """
    if name in nodes and nodes[name][0] == parameters:
        return nodes[name][1]

    remove(name)
    nodes[name] = (parameters, create())
    return nodes[name][1]


def remove(name: str) -> None:
    """
    Removes the node with the given name and all its children
    """
    for key in [key for key in nodes if key == name or key.startswith(name + "/")]:
        nodes.pop(key)[1].remove()


def frame_viz(name: str, T: np.ndarray, opacity: float = 1.0, scale: float = 1.0) -> None:
    """
    Visualizes a given frame
    """
    vis = get_viewer()
    node_name = f"frames/{name}"
    axises = {
        "x": (0xFF0000, tf.rotation_matrix(np.pi / 2, [0, 1, 0]), [1, 0, 0]),
        "y": (0x00FF00, tf.rotation_matrix(-np.pi / 2, [1, 0, 0]), [0, 1, 0]),
        "z": (0x0000FF, np.eye(4), [0, 0, 1]),
    }

    def create():
        frame = vis.scene.add_frame(node_name, show_axes=False)
        for axis_name, (color, R, direction) in axises.items():
            vis.scene.add_cylinder(
                f"{node_name}/{axis_name}",
                radius=0.005 * scale,
                height=0.1 * scale,
                color=rgb(color),
                opacity=opacity,
                wxyz=tf.quaternion_from_matrix(R),
                position=0.05 * scale * np.array(direction),
            )
        return frame

    set_pose(node(node_name, (opacity, scale), create), T)


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
    node_name = f"point/{name}"
    sphere = node(
        node_name,
        (radius, color, opacity),
        lambda: vis.scene.add_icosphere(node_name, radius=radius, color=rgb(color), opacity=opacity),
    )
    sphere.position = np.asarray(point, dtype=float)


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
    vis = get_viewer()
    node_name = f"points/{name}"
    remove(node_name)

    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(points) == 0:
        return

    sphere = trimesh.creation.icosphere(subdivisions=2, radius=radius)
    nodes[node_name] = (
        None,
        vis.scene.add_batched_meshes_simple(
            node_name,
            sphere.vertices,
            sphere.faces,
            batched_wxyzs=np.tile([1.0, 0.0, 0.0, 0.0], (len(points), 1)),
            batched_positions=points,
            batched_colors=rgb(color),
            opacity=opacity,
        ),
    )


def robot_frame_viz(robot: placo.RobotWrapper, frame: str, opacity: float = 1.0, scale: float = 1.0) -> None:
    """
    Draw a frame from the robot
    """
    node_name = f"{robot_names[robot]}_{frame}"
    frame_viz(node_name, robot.get_T_world_frame(frame), opacity, scale)


def segments_viz(node_name: str, segments: np.ndarray, color: float) -> None:
    """
    Draws line segments, given as an array of shape (N, 2, 3)
    """
    vis = get_viewer()
    remove(node_name)
    if len(segments) > 0:
        nodes[node_name] = (
            None,
            vis.scene.add_line_segments(
                node_name, segments, rgb(color), thickness=2.0, thickness_units="screen"
            ),
        )


def to_3d(points: np.ndarray) -> np.ndarray:
    """
    Points as an array of shape (N, 3), 2D points being put at z = 0
    """
    points = np.atleast_2d(np.asarray(points, dtype=float))
    if points.shape[1] == 2:
        points = np.hstack((points, np.zeros((len(points), 1))))
    return points


steps: int = 0


def footsteps_viz(footsteps: placo.Footsteps, T: np.ndarray = np.eye(4)) -> None:
    global steps

    if len(footsteps) < steps:
        remove("footsteps")
    steps = len(footsteps)

    k = 0
    for footstep in footsteps:
        k += 1
        polygon = np.array([(T @ [*xy, 0, 1])[:3] for xy in footstep.support_polygon()])

        if isinstance(footstep, placo.Support):
            if len(footstep.footsteps) >= 2:
                color = 0x1111AA
            else:
                color = 0xFF3333 if str(footstep.footsteps[0].side) == "left" else 0x33FF33
        else:
            color = 0xFF3333 if str(footstep.side) == "left" else 0x33FF33

        # Closed polygon
        segments = np.stack((polygon, np.roll(polygon, -1, axis=0)), axis=1)
        segments_viz(f"footsteps/{k}", segments, color)


def line_viz(name: str, points: np.ndarray, color: float = 0xFF0000) -> None:
    """
    Prints a line
    """
    # Points are taken by pairs, each pair being a segment
    points = to_3d(points)
    segments_viz(f"lines/{name}", points[: 2 * (len(points) // 2)].reshape(-1, 2, 3), color)


def cross_viz(name: str, position: np.ndarray, size: float = 0.15, color: float = 0xFF0000) -> None:
    """
    Prints a cross
    """
    points = np.array(
        [
            position - np.array([size, size]),
            position + np.array([size, size]),
            position - np.array([size, -size]),
            position + np.array([size, -size]),
        ]
    )
    segments_viz(f"crosses/{name}", to_3d(points).reshape(-1, 2, 3), color)


def path_viz(name: str, points: np.ndarray, color: float = 0xFF0000) -> None:
    """
    Prints a path
    """
    points = to_3d(points)
    segments_viz(f"path_{name}", np.stack((points[:-1], points[1:]), axis=1), color)


def cylinder_viz(
    name: str,
    position: np.ndarray,
    length: float,
    radius: float,
    color: float = 0xFF0000,
    opacity=1.0,
) -> None:
    vis = get_viewer()
    node_name = f"cylinders/{name}"

    # Vertical cylinder standing on the ground at the given (x, y) position
    cylinder = node(
        node_name,
        (length, radius, color, opacity),
        lambda: vis.scene.add_cylinder(node_name, radius=radius, height=length, color=rgb(color), opacity=opacity),
    )
    cylinder.position = np.array([*position, length / 2])


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
    vis = get_viewer()
    node_name = f"arrows/{name}"
    remove(node_name)

    point_from = np.asarray(point_from, dtype=float)
    point_to = np.asarray(point_to, dtype=float)
    if np.linalg.norm(point_to - point_from) < 1e-6:
        point_to = point_from + np.array([0.0, 0.0, 1e-3])

    nodes[node_name] = (
        None,
        vis.scene.add_arrows(
            node_name,
            np.array([[point_from, point_to]]),
            rgb(color),
            shaft_radius=radius,
            head_radius=2 * radius,
            head_length=3 * radius,
        ),
    )


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

    k_d = k
    while k_d < previous_contacts:
        remove(f"arrows/contact_{k_d}")
        k_d += 1
    previous_contacts = k


# Interactive controls of the tasks, by name
controls: dict = {}


def position_task_controls(task, name: str, scale: float = 0.15) -> viser.TransformControlsHandle:
    """
    Makes the target of a position task (kinematics or dynamics) editable from the viewer: a gizmo placed at the
    target can be dragged to move it.

    The target is updated by the viewer, it should not be changed by the code at the same time.
    """
    vis = get_viewer()
    node_name = f"controls/{name}"
    remove(node_name)

    handle = vis.scene.add_transform_controls(
        node_name, scale=scale, disable_rotations=True, position=np.array(task.target_world)
    )

    # Asynchronous callbacks are called in order (unlike the threaded ones)
    @handle.on_update
    async def _(event) -> None:
        task.target_world = np.array(event.target.position)

    nodes[node_name] = (None, handle)
    controls[name] = handle
    return handle


def frame_task_controls(task, name: str, scale: float = 0.15) -> viser.TransformControlsHandle:
    """
    Makes the target of a frame task (kinematics or dynamics) editable from the viewer: a gizmo placed at the target
    can be dragged and rotated to move it.

    The target is updated by the viewer, it should not be changed by the code at the same time.
    """
    vis = get_viewer()
    node_name = f"controls/{name}"
    remove(node_name)

    T = np.array(task.T_world_frame)
    handle = vis.scene.add_transform_controls(
        node_name, scale=scale, position=T[:3, 3], wxyz=tf.quaternion_from_matrix(T)
    )

    @handle.on_update
    async def _(event) -> None:
        T_world_frame = tf.quaternion_matrix(event.target.wxyz)
        T_world_frame[:3, 3] = event.target.position
        task.T_world_frame = T_world_frame

    nodes[node_name] = (None, handle)
    controls[name] = handle
    return handle


def task_controls(task, name: str, scale: float = 0.15) -> viser.TransformControlsHandle:
    """
    Makes the target of a position or frame task editable from the viewer (see position_task_controls and
    frame_task_controls)
    """
    if hasattr(task, "T_world_frame"):
        return frame_task_controls(task, name, scale)
    elif hasattr(task, "target_world"):
        return position_task_controls(task, name, scale)
    raise ValueError(f"task_controls: {type(task).__name__} is not a position or frame task")
