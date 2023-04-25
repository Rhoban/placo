#pragma once

#include "placo/control/kinematics_solver.h"
#include "placo/planning/walk_pattern_generator.h"
#include "placo/planning/kick.h"

namespace placo
{
class WalkTasks
{
public:
  void initialize_tasks(KinematicsSolver* solver);
  void remove_tasks();
  virtual ~WalkTasks();
  
  void update_tasks(WalkPatternGenerator::Trajectory& trajectory, double t);
  void update_tasks(Kick& kick, double t);
  void update_tasks(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right, Eigen::Vector3d com_world,
                    Eigen::Matrix3d R_world_trunk);

  KinematicsSolver* solver = nullptr;
  placo::FrameTask left_foot_task;
  placo::FrameTask right_foot_task;
  placo::CoMTask* com_task;
  placo::OrientationTask* trunk_orientation_task;
};
}  // namespace placo