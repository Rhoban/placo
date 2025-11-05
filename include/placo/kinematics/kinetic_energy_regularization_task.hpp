#pragma once

#include "placo/kinematics/task.hpp"

namespace placo::kinematics {
class KinematicsSolver;
struct KineticEnergyRegularizationTask : public Task {
  virtual void update();
  virtual std::string type_name();
  virtual std::string error_unit();
};
} // namespace placo::kinematics