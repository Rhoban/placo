#include "placo/humanoid/walk_tasks.h"
#include "placo/humanoid/humanoid_robot.h"
#include "pinocchio/math/rpy.hpp"

namespace placo::humanoid
{
using namespace placo::kinematics;
using namespace placo::tools;

void WalkTasks::initialize_tasks(KinematicsSolver* solver_, HumanoidRobot* robot_)
{
  robot = robot_;
  solver = solver_;

  left_foot_task = solver->add_frame_task("left_foot", robot->get_T_world_left());
  left_foot_task.configure("left_foot", scaled ? "scaled" : "soft", 1., 1.);

  right_foot_task = solver->add_frame_task("right_foot", robot->get_T_world_right());
  right_foot_task.configure("right_foot", scaled ? "scaled" : "soft", 1., 1.);

  trunk_orientation_task = &solver->add_orientation_task("trunk", robot->get_T_world_trunk().rotation());
  trunk_orientation_task->configure("trunk", scaled ? "scaled" : "soft", 1.);

  update_com_task();
}

void WalkTasks::update_com_task()
{
  if (trunk_mode)
  {
    if (com_task != nullptr)
    {
      solver->remove_task(*com_task);
      com_task = nullptr;
    }
    if (trunk_task == nullptr)
    {
      trunk_task = &solver->add_position_task("trunk", robot->get_T_world_frame("trunk").translation());
      trunk_task->configure("trunk", scaled ? "scaled" : "soft", 1.);
    }
  }
  else
  {
    if (trunk_task != nullptr)
    {
      solver->remove_task(*trunk_task);
      trunk_task = nullptr;
    }
    if (com_task == nullptr)
    {
      com_task = &solver->add_com_task(robot->com_world());
      com_task->configure("com", scaled ? "scaled" : "soft", 1.);
    }
  }
}

void WalkTasks::reach_initial_pose(Eigen::Affine3d T_world_left, double feet_spacing, double com_height,
                                   double trunk_pitch)
{
  Eigen::Affine3d T_world_right = T_world_left;
  T_world_right.translation() = T_world_left.translation() + T_world_left.rotation() * Eigen::Vector3d(0, -feet_spacing, 0);

  Eigen::Vector3d com_world = interpolate_frames(T_world_left, T_world_right, .5).translation();
  com_world.z() = com_height;

  double trunk_yaw = pinocchio::rpy::matrixToRpy(T_world_left.rotation()).z();
  Eigen::Matrix3d R_world_trunk = pinocchio::rpy::rpyToMatrix(Eigen::Vector3d(0, trunk_pitch, trunk_yaw));
  trunk_orientation_task->R_world_frame = R_world_trunk;

  update_tasks(T_world_left, T_world_right, com_world, R_world_trunk);
  
  for (int i = 0; i < 100; i++)
  {
    if (i <= 10)
    {
      // Adding noise to avoid singularities
      solver->robot.add_q_noise(0.01);
    }

    robot->update_kinematics();
    solver->solve(true);
  }
}

void WalkTasks::update_tasks(WalkPatternGenerator::Trajectory& trajectory, double t)
{
  update_tasks(trajectory.get_T_world_left(t), trajectory.get_T_world_right(t),
               trajectory.get_p_world_CoM(t + com_delay), trajectory.get_R_world_trunk(t));
}

void WalkTasks::update_tasks(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right, Eigen::Vector3d com_world,
                             Eigen::Matrix3d R_world_trunk)
{
  update_com_task();
  Eigen::Vector3d offset = robot->get_T_world_frame("trunk").linear() * Eigen::Vector3d(com_x, com_y, 0);

  if (trunk_mode)
  {
    trunk_task->target_world = com_world + offset;
  }
  else
  {
    com_task->target_world = com_world + offset;
  }

  left_foot_task.set_T_world_frame(T_world_left);
  right_foot_task.set_T_world_frame(T_world_right);
  trunk_orientation_task->R_world_frame = R_world_trunk;
}

void WalkTasks::remove_tasks()
{
  if (solver != nullptr)
  {
    solver->remove_task(left_foot_task);
    solver->remove_task(right_foot_task);
    if (com_task != nullptr)
    {
      solver->remove_task(*com_task);
      com_task = nullptr;
    }
    if (trunk_task != nullptr)
    {
      solver->remove_task(*trunk_task);
      trunk_task = nullptr;
    }
    solver->remove_task(*trunk_orientation_task);
    solver = nullptr;
  }
}

void WalkTasks::update_tasks_and_pid(WalkPatternGenerator::Trajectory& trajectory, double t, Eigen::Vector2d dcm, double omega, double elapsed)
{
  Eigen::Vector2d error = dcm - trajectory.get_p_world_DCM(t + com_delay, omega);
  integral = (1.0 - lambda) * integral + error * elapsed;
  Eigen::Vector2d derivative = (error - last_error) / elapsed;
  last_error = error;

  Eigen::Vector2d com_offset = K_p * error + K_i * integral + K_d * derivative;

  update_tasks(trajectory.get_T_world_left(t), trajectory.get_T_world_right(t),
               trajectory.get_p_world_CoM(t + com_delay) + Eigen::Vector3d(com_offset[0], com_offset[1], 0.), 
               trajectory.get_R_world_trunk(t));
}

std::map<std::string, Eigen::Vector3d> WalkTasks::get_tasks_error()
{
  std::map<std::string, Eigen::Vector3d> error;
  error["left_foot_orientation"] = left_foot_task.orientation->error();
  error["right_foot_orientation"] = right_foot_task.orientation->error();
  error["left_foot_orientation"] = left_foot_task.position->error();
  error["right_foot_orientation"] = right_foot_task.position->error();
  error["trunk_orientation"] = trunk_orientation_task->error();
  if (trunk_mode)
  {
    error["trunk_position"] = trunk_task->error();
  }
  else
  {
    error["com_position"] = com_task->error();
  }
  return error;
}

WalkTasks::~WalkTasks()
{
}
};  // namespace placo::humanoid