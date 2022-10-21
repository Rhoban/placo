# TF utilities
import numpy as np
from transforms3d import axangles, affines

def rotation(axis, angle):
    return axangles.axangle2aff(axis, angle)

def translation(xyz):
    m = np.eye(4)
    m[:3, 3] = xyz
    return m

def frame(axis, angle, xyz):
    return translation(xyz) @ rotation(axis, angle)