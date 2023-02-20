#pragma once

#include "placo/planning/solver_task_holder.h"
#include "placo/control/kinematics_solver.h"
#include "placo/model/humanoid_robot.h"

namespace placo
{
SolverTaskHolder::SolverTaskHolder(HumanoidRobot& robot, KinematicsSolver& solver)
  : robot(robot)
  , solver(solver)
  , com_task(solver.add_com_task(robot.com_world()))
  , trunk_orientation_task(solver.add_orientation_task("trunk", robot.get_T_world_trunk().rotation()))
{
  init_tasks();
}

void SolverTaskHolder::init_tasks()
{
  solver.mask_dof("head_pitch");
  solver.mask_dof("head_yaw");
  solver.mask_dof("left_elbow");
  solver.mask_dof("right_elbow");
  solver.mask_dof("left_shoulder_pitch");
  solver.mask_dof("right_shoulder_pitch");
  solver.mask_dof("left_shoulder_roll");
  solver.mask_dof("right_shoulder_roll");

  left_foot = solver.add_frame_task("left_foot", robot.get_T_world_left());
  left_foot.configure("left_foot", "soft", 1., 1.);

  right_foot = solver.add_frame_task("right_foot", robot.get_T_world_right());
  right_foot.configure("right_foot", "soft", 1., 1.);

  com_task.configure("com", "soft", 1.0);
  trunk_orientation_task.configure("trunk", "soft", 1.0);

  solver.add_regularization_task(1e-6);
  solver.solve(true);
  robot.update_kinematics();
}

void SolverTaskHolder::update_tasks(Eigen::Affine3d left_frame, Eigen::Affine3d right_frame, Eigen::Vector3d com_vector,
                                    Eigen::Matrix3d trunk_orientation, bool dump_status)
{
  left_foot.set_T_world_frame(left_frame);
  right_foot.set_T_world_frame(right_frame);
  com_task.target_world = com_vector;
  trunk_orientation_task.R_world_frame = trunk_orientation;

  solver.solve(true);
  if (dump_status)
  {
    solver.dump_status();
  }
}

void SolverTaskHolder::configure_weight(double lf, double rf, double com, double trunk)
{
  left_foot.configure("left_foot", "soft", lf, lf);
  right_foot.configure("right_foot", "soft", rf, rf);
  com_task.configure("com", "soft", com);
  trunk_orientation_task.configure("trunk", "soft", trunk);
}
}  // namespace placo