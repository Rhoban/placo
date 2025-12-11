#include "placo/kinematics/regularization_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
void RegularizationTask::set_weight(double weight)
{
  this->default_weight = weight;
  dirty = true;
}

void RegularizationTask::set_joint_weight(std::string joint, double weight)
{
  joint_weights[joint] = weight;
  dirty = true;
}

void RegularizationTask::update()
{
  // Regularization task if of the form
  // W \Delta q = 0
  // Weight is square rooted to be consistent with other tasks (W will be squared in the QP)
  if (dirty)
  {
    W = Eigen::MatrixXd(solver->N, solver->N);
    W.setIdentity();
    W *= sqrt(default_weight);

    // Per-joint overrides
    for (auto& entry : joint_weights)
    {
      int joint_id = solver->robot.get_joint_v_offset(entry.first);
      int joint_size = solver->robot.get_joint_v_size(entry.first);
      for (int i = 0; i < joint_size; i++)
      {
        W(joint_id + i, joint_id + i) = sqrt(entry.second);
      }
    }
    dirty = false;
  }

  A = Eigen::MatrixXd(solver->N - 6, solver->N);
  A.block(0, 0, solver->N - 6, solver->N) = W.block(6, 0, solver->N - 6, solver->N);

  b = Eigen::MatrixXd(solver->N - 6, 1);
  b.setZero();
}

std::string RegularizationTask::type_name()
{
  return "regularization";
}

std::string RegularizationTask::error_unit()
{
  return "none";
}
}  // namespace placo::kinematics