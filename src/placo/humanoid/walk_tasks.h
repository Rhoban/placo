#pragma once

#include "placo/kinematics/kinematics_solver.h"
#include "placo/humanoid/walk_pattern_generator.h"

namespace placo::humanoid
{
class WalkTasks
{
public:
  void initialize_tasks(placo::kinematics::KinematicsSolver* solver, placo::humanoid::HumanoidRobot* robot);
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

  placo::kinematics::CoMTask* com_task = nullptr;
  placo::kinematics::PositionTask* trunk_task = nullptr;

  void update_com_task();

  void reach_initial_pose(Eigen::Affine3d T_world_left, double feet_spacing, double com_height, double trunk_pitch);

  bool scaled = false;

  bool trunk_mode = false;
  double com_delay = 0.;
  double com_x = 0.;
  double com_y = 0.;

  // DCM error PID
  void update_tasks_and_pid(WalkPatternGenerator::Trajectory& trajectory, double t, Eigen::Vector2d dcm, double omega, double elapsed);

  double K_p = 0.;
  double K_i = 0.;
  double K_d = 0.;

  double lambda = 0.01;
  Eigen::Vector2d integral;
  Eigen::Vector2d last_error;
};
}  // namespace placo::humanoid