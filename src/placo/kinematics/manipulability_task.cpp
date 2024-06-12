#include "placo/kinematics/manipulability_task.h"
#include "placo/kinematics/kinematics_solver.h"

namespace placo::kinematics
{
ManipulabilityTask::ManipulabilityTask(model::RobotWrapper::FrameIndex frame_index, Type type, double lambda)
  : frame_index(frame_index), type(type), lambda(lambda)
{
}

Eigen::MatrixXd ManipulabilityTask::mask_matrix(Eigen::MatrixXd M)
{
  if (type == POSITION)
  {
    return M.block(0, 0, 3, M.cols());
  }
  else if (type == ORIENTATION)
  {
    return M.block(3, 0, 3, M.cols());
  }
  else
  {
    return M;
  }
}

void ManipulabilityTask::update()
{
  // Computing the Jacobian matrix
  Eigen::MatrixXd J = mask_matrix(solver->robot.frame_jacobian(frame_index));
  Eigen::MatrixXd JJT_inv = (J * J.transpose()).inverse();
  double manipulability = sqrt((J * J.transpose()).determinant());

  Eigen::VectorXd manipulability_gradient(solver->N - 6);
  manipulability_gradient.setZero();
  Eigen::VectorXd qd_save = solver->robot.state.qd;
  for (int dof = 6; dof < solver->N; dof++)
  {
    // Computing the time variation with a unitary velocity to obtain a part of the Hessian
    solver->robot.state.qd.setZero();
    solver->robot.state.qd(dof) = 1.0;
    solver->robot.update_time_variations();

    Eigen::MatrixXd H_dof = mask_matrix(solver->robot.frame_jacobian_time_variation(frame_index));
    Eigen::MatrixXd JH = J * H_dof.transpose();

    manipulability_gradient(dof - 6) = manipulability * JH.cwiseProduct(JJT_inv).sum();
  }

  solver->robot.state.qd = qd_save;
  solver->robot.update_time_variations();

  // Regularization magnitude is handled through the task weight (see add_regularization_task)
  // Floating base is not regularized by this task
  Eigen::MatrixXd I = Eigen::MatrixXd(solver->N, solver->N);
  I.setIdentity();

  A = Eigen::MatrixXd(solver->N - 6, solver->N);
  A.block(0, 0, solver->N - 6, solver->N) = I.block(6, 0, solver->N - 6, solver->N) * lambda;

  b = (1 / (2. * lambda)) * manipulability_gradient;
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