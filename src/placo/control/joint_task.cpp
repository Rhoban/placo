#include "placo/control/joint_task.h"
#include "placo/control/kinematics_solver.h"

namespace placo
{
JointTask::JointTask(std::string joint, double target) : joint(joint), target(target)
{
}

void JointTask::update()
{
  A = Eigen::MatrixXd(1, solver->N);
  A.setZero();
  A(0, solver->robot->get_joint_v_offset(joint)) = 1;

  b = Eigen::MatrixXd(1, 1);
  b(0, 0) = target - solver->robot->get_joint(joint);
}

std::string JointTask::type_name()
{
  return "joint";
}

std::string JointTask::error_unit()
{
  return "dof-rad";
}
}  // namespace placo