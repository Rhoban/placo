#pragma once

#include "placo/control/kinematics_solver.h"
#include "placo/planning/walk_pattern_generator.h"

namespace placo
{
class WalkTasks
{
public:
  void initialize_tasks(KinematicsSolver* solver);
  void remove_tasks();
  virtual ~WalkTasks();
  void update_tasks(WalkPatternGenerator::Trajectory& trajectory, double t);

  KinematicsSolver* solver = nullptr;
  placo::FrameTask left_foot_task;
  placo::FrameTask right_foot_task;
  placo::CoMTask* com_task;
  placo::OrientationTask* trunk_orientation_task;
};
}  // namespace placo