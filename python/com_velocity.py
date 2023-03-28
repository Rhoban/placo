import time
import placo
import argparse
import tf
import numpy as np
import matplotlib.pyplot as plt

from pinocchio.explog import log

# Loading the robot
robot = placo.HumanoidRobot("sigmaban")

# Walk parameters
parameters = placo.HumanoidParameters()
parameters.dt = 0.025
parameters.single_support_duration = .35
parameters.double_support_duration = 0.0
parameters.startend_double_support_duration = 0.5
parameters.planned_dt = 500
parameters.replan_frequency = 500
parameters.walk_com_height = 0.32
parameters.walk_foot_height = 0.04
parameters.pendulum_height = 0.32
parameters.walk_trunk_pitch = 0.2
parameters.walk_foot_tilt = 0.2
parameters.foot_length = 0.1576
parameters.foot_width = 0.092
parameters.feet_spacing = 0.122
parameters.zmp_margin = 0.02

# Creating the kinematics solver
solver = robot.make_solver()
task_holder = placo.SolverTaskHolder(robot, solver)

elbow = -120*np.pi/180
task_holder.update_arms_task(elbow, elbow, 0., 0., 0., 0.)
task_holder.update_head_task(0., 0.)

# Creating the FootstepsPlanner
T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
T_world_right = placo.flatten_on_floor(robot.get_T_world_right())
planner = placo.FootstepsPlannerRepetitive(parameters)
d_x = 0.1
d_y = 0.
d_theta = 0.4
nb_steps = 15
planner.configure(d_x, d_y, d_theta, nb_steps)

# Creating the walk pattern generator
walk = placo.WalkPatternGenerator(robot, parameters)

footsteps = planner.plan(placo.HumanoidRobot_Side.left,
                         T_world_left, T_world_right)

double_supports = parameters.double_support_duration / parameters.dt >= 1
supports = placo.FootstepsPlanner.make_supports(
    footsteps, True, double_supports, True)

trajectory = walk.plan(supports)

# Walk of the robot
ts = np.linspace(0, trajectory.duration, 1000)
measured_com_vel = []
measured_dcm = []
joints_speed = []

for t in ts:
    task_holder.update_walk_tasks(trajectory.get_T_world_left(t), trajectory.get_T_world_right(t),
                                  trajectory.get_p_world_CoM(t), trajectory.get_R_world_trunk(t), trajectory.duration/1000, False)

    robot.update_kinematics()
    robot.update_support_side(str(trajectory.support_side(t)))
    robot.ensure_on_floor()

    qd_a = robot.state.qd[6:]
    joints_speed.append(qd_a)

    if t > 0:
        robot.update_trunk_angular_velocity(trajectory.duration/1000)
    omega_b = robot.get_omega()

    measured_com_vel.append(robot.get_com_velocity(
        qd_a, robot.get_support_side(), omega_b))

    measured_dcm.append(
        robot.dcm(measured_com_vel[-1][:2], parameters.omega()))

# Plotting
measured_com_vel = np.array(measured_com_vel)
planned_com_vel = np.array([trajectory.com.vel(t) for t in ts])
plt.plot(ts, planned_com_vel.T[0],
         label="Planned CoM v_x", lw=3)
plt.plot(ts[10:], measured_com_vel.T[0][10:],
         label="Measured CoM v_x", lw=3)
plt.plot(ts, planned_com_vel.T[1],
         label="Planned CoM v_y", lw=3)
plt.plot(ts[10:], measured_com_vel.T[1][10:],
         label="Measured CoM v_y", lw=3)
plt.legend()
plt.grid()
plt.show()

measured_dcm = np.array(measured_dcm)
planned_dcm = np.array([trajectory.com.dcm(t) for t in ts])
plt.plot(planned_dcm.T[0][10:], planned_dcm.T[1][10:],
         label="Planned DCM trajectory", lw=3)
plt.plot(measured_dcm.T[0][10:], measured_dcm.T[1][10:],
         label="Measured DCM trajectory", lw=3)
plt.legend()
plt.grid()
plt.show()

# joints_speed = np.array(joints_speed)
# for i in range(20):
#     plt.plot(ts, joints_speed.T[0])
# plt.show()
