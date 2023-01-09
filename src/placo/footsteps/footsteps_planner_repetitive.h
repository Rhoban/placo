#pragma once

#include "footsteps_planner.h"
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace placo
{
class FootstepsPlannerRepetitive : public FootstepsPlanner
{
public:
  // FootstepsPlannerRepetitive();

  virtual std::vector<Footstep> plan();
};
}  // namespace placo