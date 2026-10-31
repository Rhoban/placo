import os
import viser
import trimesh
import numpy as np
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

        ground_viz()

    return viewer


ground = None
ground_checkbox = None


def ground_viz(z: float = 0.0, visible: bool = True) -> None:
    """
    Shows (or hides) the ground: a grid (10 cm cells, 1 m sections) at the given height. It is shown by
    default, and can also be hidden from the viewer ("Ground" checkbox)
    """
    global ground, ground_checkbox
    vis = get_viewer()

    if ground is None:
        ground = vis.scene.add_grid(
            "ground",
            infinite_grid=True,
            cell_size=0.1,
            section_size=1.0,
            cell_color=(190, 190, 190),
            section_color=(120, 120, 120),
            # Only the grid lines: no plane or shadows, so that what is below the ground stays visible
            plane_opacity=0.0,
            shadow_opacity=0.0,
            fade_distance=30.0,
        )
        ground_checkbox = vis.gui.add_checkbox("Ground", initial_value=visible)

        @ground_checkbox.on_update
        def _(_) -> None:
            ground.visible = ground_checkbox.value

    ground.position = (0.0, 0.0, z)
    ground.visible = visible
    ground_checkbox.value = visible


# Meshes loaded from files, by path
meshes: dict = {}


# Angle between faces above which an edge is sharp (not smoothed when shading)
sharp_edge_angle: float = np.radians(30)


def smooth_shaded(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """
    The mesh, with vertices duplicated along its sharp edges. The shading is smooth on curved surfaces, while sharp
    edges stay sharp (meshes loaded from CAD files would otherwise look blurry and blotchy, their merged vertices being
    shared by faces with very different orientations)
    """
    return trimesh.graph.smooth_shade(mesh, angle=sharp_edge_angle)


def load_mesh(path: str, scene: bool):
    """
    Loads a mesh file (once), as a trimesh Scene (keeping the colors of the file) or as a single Trimesh
    """
    key = (path, scene)
    if key not in meshes:
        if scene:
            # trimesh.load (rather than load_scene / load_mesh, which behave differently before trimesh 5)
            loaded = trimesh.load(path, force="scene")
            for name, geometry in list(loaded.geometry.items()):
                if isinstance(geometry, trimesh.Trimesh):
                    loaded.geometry[name] = smooth_shaded(geometry)
        else:
            loaded = smooth_shaded(trimesh.load(path, force="mesh"))
        meshes[key] = loaded
    return meshes[key].copy()


class RobotViz:
    """
    Displays the visual geometries of a robot in the viser viewer
    """

    def __init__(self, robot: placo.RobotWrapper, name: str):
        self.robot = robot
        self.handles = []
        vis = get_viewer()

        for geometry in robot.visual_model.objects:
            node_name = f"{name}/visual/{geometry.name}"
            color = tuple(geometry.color[:3])
            opacity = float(geometry.color[3])
            size = geometry.size

            if geometry.type == "box":
                handle = vis.scene.add_box(node_name, dimensions=tuple(size), color=color, opacity=opacity)
            elif geometry.type == "sphere":
                handle = vis.scene.add_icosphere(node_name, radius=size[0], color=color, opacity=opacity)
            elif geometry.type == "cylinder":
                handle = vis.scene.add_cylinder(node_name, radius=size[0], height=size[1], color=color, opacity=opacity)
            elif geometry.type == "ellipsoid":
                handle = vis.scene.add_icosphere(
                    node_name, radius=1.0, scale=tuple(size), color=color, opacity=opacity
                )
            elif geometry.type == "plane":
                handle = vis.scene.add_grid(node_name, width=2 * size[0], height=2 * size[1])
            elif geometry.type == "capsule":
                capsule = trimesh.creation.capsule(height=size[1], radius=size[0])
                handle = vis.scene.add_mesh_simple(
                    node_name, capsule.vertices, capsule.faces, color=color, opacity=opacity
                )
            else:
                # Meshes keep the colors of their files (e.g. Collada), unless a color is given in the model
                scale = np.array(geometry.mesh_scale)
                if os.path.splitext(geometry.mesh_path)[1].lower() == ".dae" and not geometry.has_color:
                    mesh = load_mesh(geometry.mesh_path, True)
                    mesh.apply_scale(scale)
                    handle = vis.scene.add_mesh_trimesh(node_name, mesh)
                else:
                    mesh = load_mesh(geometry.mesh_path, False)
                    mesh.apply_scale(scale)
                    handle = vis.scene.add_mesh_simple(
                        node_name, mesh.vertices, mesh.faces, color=color, opacity=opacity
                    )

            self.handles.append(handle)

        self.display(robot.state.q)

    def display(self, q: np.ndarray) -> None:
        """
        Displays the robot in the configuration q
        """
        for handle, T in zip(self.handles, self.robot.visual_placements(q)):
            set_pose(handle, T)


def robot_viz(robot: placo.RobotWrapper, name: str = "robot") -> RobotViz:
    """
    Builds the visualization of a robot in the viser viewer

    The robot can further be displayed using:

    > viz.display(q)
    """
    global robot_names

    robot_names[robot] = name
    return RobotViz(robot, name)


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
    parameters. A node is replaced atomically (the viewer never shows a frame without it, which would make it blink)
    """
    if name in nodes and nodes[name][0] == parameters:
        return nodes[name][1]

    with get_viewer().atomic():
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

    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(points) == 0:
        remove(node_name)
        return

    def create():
        sphere = trimesh.creation.icosphere(subdivisions=2, radius=radius)
        return vis.scene.add_batched_meshes_simple(
            node_name,
            sphere.vertices,
            sphere.faces,
            batched_wxyzs=np.tile([1.0, 0.0, 0.0, 0.0], (len(points), 1)),
            batched_positions=points,
            batched_colors=rgb(color),
            opacity=opacity,
        )

    # The points are moved when their number doesn't change (the node is replaced otherwise)
    node(node_name, (radius, color, opacity, len(points)), create).batched_positions = points


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
    if len(segments) == 0:
        remove(node_name)
        return

    # The segments are moved when their number doesn't change (the node is replaced otherwise)
    create = lambda: vis.scene.add_line_segments(
        node_name, segments, rgb(color), thickness=2.0, thickness_units="screen"
    )
    node(node_name, (color, len(segments)), create).points = segments


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

    point_from = np.asarray(point_from, dtype=float)
    point_to = np.asarray(point_to, dtype=float)
    if np.linalg.norm(point_to - point_from) < 1e-6:
        point_to = point_from + np.array([0.0, 0.0, 1e-3])
    points = np.array([[point_from, point_to]])

    create = lambda: vis.scene.add_arrows(
        node_name,
        points,
        rgb(color),
        shaft_radius=radius,
        head_radius=2 * radius,
        head_length=3 * radius,
    )
    node(node_name, (color, radius), create).points = points


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


# Sliders of the viewer panel, by name
sliders: dict = {}


def add_slider(name: str, minimum: float, maximum: float, value: float = None, step: float = 0.01):
    """
    Adds a slider to the viewer panel, whose current value is slider.value. If a slider with this name was already
    added, it is returned (so that this function can be called in a loop)

    :param name: slider name (displayed)
    :param minimum: minimum value
    :param maximum: maximum value
    :param value: initial value (the minimum by default)
    :param step: step between values
    :return: the slider (viser.GuiSliderHandle)
    """
    if name not in sliders:
        initial = minimum if value is None else float(np.clip(value, minimum, maximum))
        sliders[name] = get_viewer().gui.add_slider(name, minimum, maximum, step, initial)
    return sliders[name]


def joints_sliders(robot: placo.RobotWrapper, joints: list = None, step: float = 0.01) -> dict:
    """
    Adds a slider for each joint of the robot to the viewer panel (in a "Joints" folder), within the joint limits
    (unbounded joints are given [-pi, pi]), starting at the current joint value

    :param robot: the robot
    :param joints: names of the joints (all the joints by default)
    :param step: step between values
    :return: the sliders, by joint name (their current values are sliders[joint].value)
    """
    if joints is None:
        joints = list(robot.joint_names())

    result = {}
    with get_viewer().gui.add_folder("Joints"):
        for joint in joints:
            lower, upper = robot.get_joint_limits(joint)
            if not (np.isfinite(lower) and np.isfinite(upper)) or upper - lower > 1e6:
                lower, upper = -np.pi, np.pi
            result[joint] = add_slider(joint, float(lower), float(upper), robot.get_joint(joint), step)
    return result
