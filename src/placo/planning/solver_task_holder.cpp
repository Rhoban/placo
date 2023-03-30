#pragma once

#include "placo/planning/solver_task_holder.h"
#include "placo/control/kinematics_solver.h"
#include "placo/model/humanoid_robot.h"

namespace placo
{
SolverTaskHolder::SolverTaskHolder(HumanoidRobot& robot_, KinematicsSolver& solver_)
  : robot(&robot_)
  , solver(&solver_)
  , arms_task(nullptr)
  , head_task(nullptr)
  , com_task(solver_.add_com_task(robot_.com_world()))
  , trunk_orientation_task(solver_.add_orientation_task("trunk", robot_.get_T_world_trunk().rotation()))
{
  init_tasks();
}

SolverTaskHolder::SolverTaskHolder(HumanoidRobot* robot_, KinematicsSolver* solver_)
  : robot(robot_)
  , solver(solver_)
  , arms_task(nullptr)
  , head_task(nullptr)
  , com_task(solver_->add_com_task(robot_->com_world()))
  , trunk_orientation_task(solver_->add_orientation_task("trunk", robot_->get_T_world_trunk().rotation()))
{
  init_tasks();
}

void SolverTaskHolder::init_tasks()
{
  arms_task = &solver->add_joints_task(arms_joints);
  arms_task->configure("arms", "soft", 1.);

  head_task = &solver->add_joints_task(head_joints);
  head_task->configure("head", "soft", 1.);

  left_foot_task = solver->add_frame_task("left_foot", robot->get_T_world_left());
  left_foot_task.configure("left_foot", "soft", 1., 1.);

  right_foot_task = solver->add_frame_task("right_foot", robot->get_T_world_right());
  right_foot_task.configure("right_foot", "soft", 1., 1.);

  com_task.configure("com", "soft", 1.);
  trunk_orientation_task.configure("trunk", "soft", 1.);

  solver->add_regularization_task(1e-6);

  solver->solve(true);
  robot->update_kinematics();
}

void SolverTaskHolder::update_walk_tasks(Eigen::Affine3d left_frame, Eigen::Affine3d right_frame,
                                         Eigen::Vector3d com_vector, Eigen::Matrix3d trunk_orientation, double elapsed,
                                         double dump_status)
{
  left_foot_task.set_T_world_frame(left_frame);
  right_foot_task.set_T_world_frame(right_frame);
  com_task.target_world = com_vector;
  trunk_orientation_task.R_world_frame = trunk_orientation;

  solver->solve(true, elapsed);
  if (dump_status)
  {
    solver->dump_status();
  }
  robot->update_kinematics();
}

void SolverTaskHolder::update_head_task(double pitch, double yaw)
{
  head_task->set_joint("head_pitch", pitch);
  head_task->set_joint("head_yaw", yaw);
}

void SolverTaskHolder::update_arms_task(std::map<std::string, double> joints)
{
  for (auto dof : joints)
  {
    arms_task->set_joint(dof.first, dof.second);
  }
}

void SolverTaskHolder::update_arms_task(double l_elbow, double r_elbow, double l_shoulder_pitch,
                                        double r_shoulder_pitch, double l_shoulder_roll, double r_shoulder_roll)
{
  arms_task->set_joint("left_elbow", l_elbow);
  arms_task->set_joint("right_elbow", r_elbow);
  arms_task->set_joint("left_shoulder_pitch", l_shoulder_pitch);
  arms_task->set_joint("right_shoulder_pitch", r_shoulder_pitch);
  arms_task->set_joint("left_shoulder_roll", l_shoulder_roll);
  arms_task->set_joint("right_shoulder_roll", r_shoulder_roll);
}

void SolverTaskHolder::update_arms_task_python_binding(double l_elbow, double r_elbow, double l_shoulder_pitch,
                                                       double r_shoulder_pitch, double l_shoulder_roll,
                                                       double r_shoulder_roll)
{
  update_arms_task(l_elbow, r_elbow, l_shoulder_pitch, r_shoulder_pitch, l_shoulder_roll, r_shoulder_roll);
}

void SolverTaskHolder::configure_weight(double lf, double rf, double com, double trunk)
{
  left_foot_task.configure("left_foot", "soft", lf, lf);
  right_foot_task.configure("right_foot", "soft", rf, rf);
  com_task.configure("com", "soft", com);
  trunk_orientation_task.configure("trunk", "soft", trunk);
}
}  // namespace placo