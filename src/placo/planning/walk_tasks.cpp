#include "placo/planning/walk_tasks.h"
#include "placo/model/humanoid_robot.h"
#include "placo/utils.h"

namespace placo
{
void WalkTasks::initialize_tasks(KinematicsSolver* solver_, HumanoidRobot* robot_, HumanoidParameters* params_)
{
  left_foot_task = solver->add_frame_task("left_foot", robot->get_T_world_left());
  left_foot_task.configure("left_foot", "soft", 1., 1.);

  right_foot_task = solver->add_frame_task("right_foot", robot->get_T_world_right());
  right_foot_task.configure("right_foot", "soft", 1., 1.);

  trunk_orientation_task = &solver->add_orientation_task("trunk", robot->get_T_world_trunk().rotation());
  trunk_orientation_task->configure("trunk", "soft", 1.);

  update_com_task();

  if (adaptative_velocity_limits)
  {
    solver->enable_velocity_limits(true);
  }
}

void WalkTasks::update_com_task()
{
  if (trunk_mode)
  {
    if (com_task != nullptr)
    {
      solver->remove_task(com_task);
      com_task = nullptr;
    }
    if (trunk_task == nullptr)
    {
      trunk_task = &solver->add_position_task("trunk", robot->get_T_world_frame("trunk").translation());
      trunk_task->configure("trunk", "soft", 1.);
    }
  }
  else
  {
    if (trunk_task != nullptr)
    {
      solver->remove_task(trunk_task);
      trunk_task = nullptr;
    }
    if (com_task == nullptr)
    {
      com_task = &solver->add_com_task(robot->com_world());
      com_task->configure("com", "soft", 1.);

      com_ub_task = &solver->add_com_ub_task(parameters->walk_max_com_height);
      com_ub_task->configure("com_ub", "hard", 0);
      com_lb_task = &solver->add_com_lb_task(parameters->walk_min_com_height);
      com_lb_task->configure("com_lb", "hard", 0);
    }
  }
}

void WalkTasks::update_tasks(WalkPatternGenerator::Trajectory& trajectory, double t)
{
  // update_tasks(trajectory.get_T_world_left(t), trajectory.get_T_world_right(t),
  //              trajectory.get_p_world_CoM(t + com_delay), trajectory.get_R_world_trunk(t));
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

  if (adaptative_velocity_limits)
  {
    Eigen::VectorXd torques = Eigen::VectorXd::Zero(solver->N + 6);
    if (!robot->support_is_both)
    {
      torques = robot->static_gravity_compensation_torques(robot->support_frame());
    }

    for (auto dof : robot->actuated_joint_names())
    {
      solver->enable_velocity_limits(true);
      double expected_torque = std::abs(torques[robot->get_joint_v_offset(dof)]) + 0.1; // 0.1 is a safety margin
      double limit = velocity_limit(expected_torque, dof, use_doc_limits);
      // double limit = .1;
      robot->set_velocity_limit(dof, limit); 
    }
  }
}

void WalkTasks::remove_tasks()
{
  if (solver != nullptr)
  {
    solver->remove_task(left_foot_task);
    solver->remove_task(right_foot_task);
    if (com_task != nullptr)
    {
      solver->remove_task(com_task);
      com_task = nullptr;
    }
    if (trunk_task != nullptr)
    {
      solver->remove_task(trunk_task);
      trunk_task = nullptr;
    }
    solver->remove_task(trunk_orientation_task);
    solver = nullptr;
  }
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
  remove_tasks();
}
};  // namespace placo