import numpy as np
import time
import matplotlib.pyplot as plt
import placo
from placo import ConstraintType
import numpy as np

dt = 0.1

jerk_planner = placo.JerkPlanner(100, np.array([0., 0., 0., 0., 0., 0.]), dt, 0.)
jerk_planner.add_equality_constraint(100, np.array([1., 1]), ConstraintType.position)
jerk_planner.add_equality_constraint(100, np.array([0., 0]), ConstraintType.velocity)
jerk_planner.add_equality_constraint(100, np.array([0., 0]), ConstraintType.acceleration)

limits = jerk_planner.add_limit_constraint(.35, ConstraintType.velocity)

ineq = jerk_planner.add_lower_than_constraint(75, np.array([10, .5]), ConstraintType.position)

polygon1 = np.array([
    [-1, 0.5],
    [-.5, 1],
    [-.5, 0.65],
])

constraint1 = jerk_planner.add_polygon_constraint(50, polygon1, ConstraintType.position, 0.1)

polygon2 = np.array([
    [-1.5, 0.25],
    [-.5, 1.5],
    [-.25, 0.25],
])

constraint2 = jerk_planner.add_polygon_constraint(50, polygon2, ConstraintType.position, 0.1)

print(jerk_planner)

start = time.time()
trajectory = jerk_planner.plan()
elapsed = time.time() - start

print(f"Computation time: {elapsed*1e6}µs")
print("Limits: ", limits.is_active())
print("Ineq: ", ineq.is_active())
print("Constraint for polygon1: ", constraint1.is_active())
print("Constraint for polygon2: ", constraint2.is_active())

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

polygon1 = np.concatenate((polygon1, [polygon1[0]]))
plt.plot(polygon1.T[0], polygon1.T[1], c='red')

polygon2 = np.concatenate((polygon2, [polygon2[0]]))
plt.plot(polygon2.T[0], polygon2.T[1], c='orange')

plt.scatter(data.T[0][0][50:51], data.T[1][0][50:51], c='green')

plt.grid()
plt.axis("equal")
plt.show()
