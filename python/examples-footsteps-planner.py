import numpy as np
from transforms3d import axangles
import placo
import numpy as np

# Creating initial and target
feet_spacing = .2
T_world_left = np.eye(4)
T_world_right = np.eye(4)
T_world_right[1, 3] = -feet_spacing

T_world_targetLeft = T_world_left.copy()
T_world_targetRight = T_world_right.copy()
T_world_targetLeft[0, 3] += 1.
T_world_targetRight[0, 3] += 1.

planner = placo.FootstepsPlanner("left", placo.frame(T_world_left), placo.frame(T_world_right), feet_spacing)

result = planner.plan(placo.frame(T_world_targetLeft), placo.frame(T_world_targetRight))

for footstep in result:
    print(footstep.frame)

