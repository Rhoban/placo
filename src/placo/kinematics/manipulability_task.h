#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct ManipulabilityTask : public Task
{
  enum Type
  {
    POSITION = 0,
    ORIENTATION = 1,
    BOTH = 2
  };

  ManipulabilityTask(model::RobotWrapper::FrameIndex frame_index, Type type, double lambda_ = 1.0);

  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  Eigen::MatrixXd mask_matrix(Eigen::MatrixXd M);

  /**
   * @brief Index of the frame we want to set manipulability
   */
  model::RobotWrapper::FrameIndex frame_index;

  /**
   * @brief Importance of the hessian regularization
   */
  double lambda;

  /**
   * @brief Type of frame manipulability to compute
   */
  Type type;

  /**
   * @brief Should the manipulability be minimized (can be useful to find singularities)
   */
  bool minimize = false;

  /**
   * @brief The last computed manipulability value
   */
  double manipulability = 0.;
};
}  // namespace placo::kinematics