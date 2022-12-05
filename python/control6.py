import numpy as np
from scipy.linalg import logm, expm

model = [
    # Axis            Relative position
    [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
    [[0.0, 1.0, 0.0], [0.32, 0.0, 0.68]],
    [[0.0, 1.0, 0.0], [0.0, 0.0, 0.975]],
    [[1.0, 0.0, 0.0], [0.0, 0.0, 0.2]],
    [[0.0, 1.0, 0.0], [0.887, 0.0, 0.0]],
    [[1.0, 0.0, 0.0], [0.2, 0.0, 0.0]],
]


def skew(v):
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ]
    )


def actionMatrix(T):
    R = T[:3, :3]
    t = T[:3, 3]
    row1 = np.hstack((R, skew(t) @ R))
    row2 = np.hstack((np.zeros((3, 3)), R))

    return np.vstack((row1, row2))


def log6(M):
    L = logm(M)
    return np.array([*L[:3, 3], L[2, 1], L[0, 2], L[1, 0]])


def exp6(t):
    T = np.zeros((4, 4))
    T[:3, :3] = skew(t[3:])
    T[:3, 3] = t[:3]
    return expm(T)


screw_axises = []
M = np.eye(4)
for axis, relative_position in model:
    M[:3, 3] += relative_position
    screw_motor = np.array([0.0, 0.0, 0.0, *axis])
    screw_axises.append(actionMatrix(M) @ screw_motor)


def fk_jacobian(thetas):
    T = np.eye(4)
    J = []
    for screw_axis, theta in zip(screw_axises, thetas):
        J.append(actionMatrix(T) @ screw_axis)
        T = T @ exp6(screw_axis * theta)
    T = T @ M
    J = np.vstack(J).T

    return T, J


def inverse_target(x, y, z, yaw, pitch, roll):
    T = exp6(np.array([x, y, z, 0.0, 0.0, 0.0]))
    T = T @ exp6(np.array([0.0, 0.0, 0.0, 0.0, 0.0, yaw]))
    T = T @ exp6(np.array([0.0, 0.0, 0.0, 0.0, pitch, 0.0]))
    T = T @ exp6(np.array([0.0, 0.0, 0.0, roll, 0.0, 0.0]))
    return T


def ik(thetas, T_world_target):
    T, J = fk_jacobian(thetas)
    error = log6(T_world_target @ np.linalg.inv(T))
    return np.array(thetas) + np.linalg.pinv(J) @ error
