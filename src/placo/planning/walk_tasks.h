#pragma once

#include "placo/control/kinematics_solver.h"
#include "placo/planning/walk_pattern_generator.h"

namespace placo
{
class WalkTasks
{
public:
  void initialize_tasks(KinematicsSolver* solver, HumanoidRobot* robot, double com_z_min = -1, double com_z_max = -1);
  void remove_tasks();
  virtual ~WalkTasks();

  void update_tasks(WalkPatternGenerator::Trajectory& trajectory, double t);
  void update_tasks(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right, Eigen::Vector3d com_world, Eigen::Matrix3d R_world_trunk);

  std::map<std::string, Eigen::Vector3d> get_tasks_error();

  KinematicsSolver* solver = nullptr;
  HumanoidRobot* robot = nullptr;

  placo::FrameTask left_foot_task;
  placo::FrameTask right_foot_task;
  placo::OrientationTask* trunk_orientation_task;

  placo::CoMTask* com_task = nullptr;
  placo::CoMBoundTask* com_lb_task = nullptr;
  placo::CoMBoundTask* com_ub_task = nullptr;

  placo::PositionTask* trunk_task = nullptr;

  void update_com_task();

  void reach_initial_pose(Eigen::Affine3d T_world_left, double feet_spacing, double com_height, double trunk_pitch);

  bool adaptative_velocity_limits = false;
  bool use_doc_limits = false;

  bool trunk_mode = false;
  double com_delay = 0.;
  double com_x = 0.;
  double com_y = 0.;
};
}  // namespace placo