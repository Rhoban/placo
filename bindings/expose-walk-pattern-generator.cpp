#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/planning/walk_pattern_generator.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeWalkPatternGenerator()
{
  class_<WalkPatternGenerator::Trajectory>("WalkTrajectory")
      .add_property("footsteps", &WalkPatternGenerator::Trajectory::footsteps)
      .add_property("com", &WalkPatternGenerator::Trajectory::com);

  class_<WalkPatternGenerator>("WalkPatternGenerator", init<HumanoidRobot&>())
      .add_property("dt", &WalkPatternGenerator::dt, &WalkPatternGenerator::dt)
      .add_property("omega", &WalkPatternGenerator::omega, &WalkPatternGenerator::omega)
      .add_property("single_support_duration", &WalkPatternGenerator::single_support_duration,
                    &WalkPatternGenerator::single_support_duration)
      .add_property("double_support_duration", &WalkPatternGenerator::double_support_duration,
                    &WalkPatternGenerator::double_support_duration)
      .add_property("maximum_steps", &WalkPatternGenerator::maximum_steps, &WalkPatternGenerator::maximum_steps)
      .def("plan", &WalkPatternGenerator::plan);
}