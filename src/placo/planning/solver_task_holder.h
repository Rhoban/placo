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

  // Update the walk related tasks and solve the QP problem of the IK
  void update_walk_tasks(Eigen::Affine3d left_frame, Eigen::Affine3d right_frame, Eigen::Vector3d com_vector,
                         Eigen::Matrix3d trunk_orientation, double dump_status = false);

  void update_head_task(double pitch, double yaw);

  void update_arms_task(std::map<std::string, double> joints);
  void update_arms_task(double l_elbow, double r_elbow, double l_shoulder_pitch, double r_shoulder_pitch,
                        double l_shoulder_roll, double r_shoulder_roll);

  void update_arms_task_python_binding(double l_elbow, double r_elbow, double l_shoulder_pitch, double r_shoulder_pitch,
                                       double l_shoulder_roll, double r_shoulder_roll);

  void configure_weight(double lf = 1.0, double rf = 1.0, double com = 1.0, double trunk = 1.0);

protected:
  // Robot
  HumanoidRobot* robot;

  // Kinematic solver
  KinematicsSolver* solver;

  FrameTask left_foot_task;
  FrameTask right_foot_task;
  JointsTask* arms_task;
  JointsTask* head_task;
  CoMTask& com_task;
  OrientationTask& trunk_orientation_task;

  std::map<std::string, double> arms_joints = { { "left_elbow", 0. },          { "right_elbow", 0. },
                                                { "left_shoulder_pitch", 0. }, { "right_shoulder_pitch", 0. },
                                                { "left_shoulder_roll", 0. },  { "right_shoulder_roll", 0. } };

  std::map<std::string, double> head_joints = { { "head_pitch", 0. }, { "head_yaw", 0. } };
};
}  // namespace placo