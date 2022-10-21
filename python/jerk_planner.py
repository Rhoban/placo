import numpy as np
import time
import matplotlib.pyplot as plt
import placo
from placo import ConstraintType
import tf
import numpy as np
import argparse

dt = 0.1
jerk_planner = placo.JerkPlanner(100, np.array([0., 0., 0., 0., 0., 0.]), dt, 0.)
jerk_planner.add_equality_constraint(100, np.array([1., 1]), ConstraintType.position)
jerk_planner.add_equality_constraint(100, np.array([0., 0]), ConstraintType.velocity)
jerk_planner.add_equality_constraint(100, np.array([0., 0]), ConstraintType.acceleration)

polygon = np.array([
    [-1, 0.5],
    [-.5, 1],
    [-.5, 0.65],
])

jerk_planner.add_inequality_polygon_constraint(50, polygon, ConstraintType.position, 0.1)

trajectory = jerk_planner.plan()

# Gathering 100 data points along the trajectory
ts = np.linspace(0, trajectory.duration(), 100)
data = np.array([[trajectory.pos(t), trajectory.vel(t), trajectory.acc(t)] for t in ts])

# Pos, speed and vel for x and y
for name, axis in ['x', 0], ['y', 1]:
    plt.plot(ts, data.T[axis][0], label=f"{name}_pos")
    plt.plot(ts, data.T[axis][1], label=f"{name}_vel")
    plt.plot(ts, data.T[axis][2], label=f"{name}_acc")
    plt.grid()
    plt.legend()
    plt.show()

# Plotting 2D trajectory
plt.scatter(data.T[0][0], data.T[1][0])

polygon = np.concatenate((polygon, [polygon[0]]))
plt.plot(polygon.T[0], polygon.T[1], c='red')

plt.grid()
plt.axis("equal")
plt.show()
