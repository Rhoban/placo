import numpy as np
from transforms3d import axangles
import placo

print("* Creating a frame")
frame = placo.Affine3d.from_matrix(np.eye(4))
print(frame)

print("* Setting translation to 1,2,3 and rotation to 1 rad about z axis")
frame.t = np.array([1, 2, 3])
frame.R = axangles.axangle2mat((0, 0, 1), 1)
print(frame)

print("* Frame times point:")
print(frame * np.array([1, 2, 3]))

print("* Two frames multiplication: ")
print(frame * frame)