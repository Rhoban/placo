#pragma once

#include "placo/kinematics/task.h"

namespace placo::kinematics
{
class KinematicsSolver;
struct RegularizationTask : public Task
{
  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();

  /**
   * @brief Set the weight for all joints
   * @param weight 
   */
  void set_weight(double weight);

  /**
   * @brief Set a joint-specific weight
   * @param joint 
   * @param weight 
   */
  void set_joint_weight(std::string joint, double weight);

protected:
  /**
   * @brief Per-joint weights
   */
  std::map<std::string, double> joint_weights;

  /**
   * @brief Default weight for all joints
   */
  double default_weight = 1.0;

  /**
   * @brief Indicates whether W needs to be recomputed
   */
  bool dirty = true;

  /**
   * @brief Matrix such that W \Delta q = 0 is the regularization task
   */
  Eigen::MatrixXd W;
};
}  // namespace placo::kinematics