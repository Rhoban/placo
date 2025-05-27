#include "placo/kinematics/manipulability_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
ManipulabilityTask::ManipulabilityTask(model::RobotWrapper::FrameIndex frame_index, Type type, double lambda)
  : frame_index(frame_index), lambda(lambda), type(type)
{
}

Eigen::MatrixXd ManipulabilityTask::mask_matrix(Eigen::MatrixXd M)
{
  if (type == POSITION)
  {
    return M.block(0, 6, 3, M.cols() - 6);
  }
  else if (type == ORIENTATION)
  {
    return M.block(3, 6, 3, M.cols() - 6);
  }
  else
  {
    return M.block(0, 6, 6, M.cols() - 6);
  }
}

void ManipulabilityTask::update()
{
  // Computing the Jacobian matrix
  Eigen::MatrixXd J_unmasked = solver->robot.frame_jacobian(frame_index, pinocchio::LOCAL);
  Eigen::MatrixXd J = mask_matrix(J_unmasked);
  Eigen::MatrixXd JJT_inv = (J * J.transpose()).inverse();
  manipulability = sqrt(fmax(0., (J * J.transpose()).determinant()));
  solver->robot.compute_hessians();

  Eigen::VectorXd manipulability_gradient(solver->N - 6);
  manipulability_gradient.setZero();

  for (int dof = 6; dof < solver->N; dof++)
  {
    Eigen::MatrixXd H_dof_unmasked = solver->robot.get_frame_hessian(frame_index, dof);
    Eigen::MatrixXd H_dof = mask_matrix(H_dof_unmasked);
    Eigen::MatrixXd JH = J * H_dof.transpose();

    manipulability_gradient(dof - 6) = manipulability * JH.cwiseProduct(JJT_inv).sum();
  }

  // Regularization magnitude is handled through the task weight (see add_regularization_task)
  // Floating base is not regularized by this task
  Eigen::MatrixXd I = Eigen::MatrixXd(solver->N, solver->N);
  I.setIdentity();

  A = Eigen::MatrixXd(solver->N - 6, solver->N);
  A.block(0, 0, solver->N - 6, solver->N) = I.block(6, 0, solver->N - 6, solver->N) * lambda;

  b = (1 / (2. * lambda)) * manipulability_gradient;

  if (minimize)
  {
    b = -b;
  }
}

std::string ManipulabilityTask::type_name()
{
  return "manipulability";
}

std::string ManipulabilityTask::error_unit()
{
  return "none";
}
}  // namespace placo::kinematics