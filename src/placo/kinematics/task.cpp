#include "placo/kinematics/task.hpp"

namespace placo::kinematics {
Eigen::MatrixXd Task::error() { return b; }

double Task::error_norm() { return b.norm(); }
} // namespace placo::kinematics