# TF utilities
import numpy as np
from transforms3d import axangles, affines


def rotation(axis: list, angle: float) -> np.ndarray:
    return axangles.axangle2aff(axis, angle)


def translation(xyz: list) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = xyz
    return m


def frame(
    axis: list = [0.0, 0.0, 1.0], angle: float = 0.0, xyz: list = [0.0, 0.0, 0.0]
) -> np.ndarray:
    return translation(xyz) @ rotation(axis, angle)
