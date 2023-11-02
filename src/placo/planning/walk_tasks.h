#pragma once

#include "placo/kinematics/kinematics_solver.h"
#include "placo/planning/walk_pattern_generator.h"

namespace placo
{
class WalkTasks
{
public:
  void initialize_tasks(placo::kinematics::KinematicsSolver* solver, HumanoidRobot* robot, double com_z_min = -1,
                        double com_z_max = -1);
  void remove_tasks();
  virtual ~WalkTasks();

  void update_tasks(WalkPatternGenerator::Trajectory& trajectory, double t);
  void update_tasks(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right, Eigen::Vector3d com_world,
                    Eigen::Matrix3d R_world_trunk);

  std::map<std::string, Eigen::Vector3d> get_tasks_error();

  placo::kinematics::KinematicsSolver* solver = nullptr;
  HumanoidRobot* robot = nullptr;

  placo::kinematics::FrameTask left_foot_task;
  placo::kinematics::FrameTask right_foot_task;
  placo::kinematics::OrientationTask* trunk_orientation_task;

  placo::kinematics::CoMTask* com_xy_task = nullptr;
  placo::kinematics::CoMTask* com_z_task = nullptr;
  placo::kinematics::CoMBoundTask* com_lb_task = nullptr;
  placo::kinematics::CoMBoundTask* com_ub_task = nullptr;

  placo::kinematics::PositionTask* trunk_task = nullptr;

  bool relax_com_height = false;
  bool relax_trunk_orientation = false;
  double relax_weight = 1e-3;

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