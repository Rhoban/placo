#pragma once

#include "placo/control/kinematics_solver.h"
#include "placo/model/humanoid_robot.h"

namespace placo
{
class SolverTaskHolder
{
public:
  SolverTaskHolder(HumanoidRobot& robot_, KinematicsSolver& solver_);
  SolverTaskHolder(HumanoidRobot* robot_, KinematicsSolver* solver_);

  void init_tasks();

  void update_tasks(Eigen::Affine3d left_frame, Eigen::Affine3d right_frame, Eigen::Vector3d com_vector,
                    Eigen::Matrix3d trunk_orientation, bool dump_status = false);

  void configure_weight(double lf = 1.0, double rf = 1.0, double com = 1.0, double trunk = 1.0);

protected:
  // Robot
  HumanoidRobot* robot;

  // Kinematic solver
  KinematicsSolver* solver;

  FrameTask left_foot;
  FrameTask right_foot;
  CoMTask& com_task;
  OrientationTask& trunk_orientation_task;
};
}  // namespace placo