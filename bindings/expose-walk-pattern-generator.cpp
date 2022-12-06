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
      .add_property("com", &WalkPatternGenerator::Trajectory::com)
      .add_property("duration", &WalkPatternGenerator::Trajectory::duration)
      .def("get_T_world_left", &WalkPatternGenerator::Trajectory::get_T_world_left)
      .def("get_T_world_right", &WalkPatternGenerator::Trajectory::get_T_world_right);

  class_<WalkPatternGenerator>("WalkPatternGenerator", init<HumanoidRobot&>())
      .add_property("parameters", &WalkPatternGenerator::parameters, &WalkPatternGenerator::parameters)
      .def("plan", &WalkPatternGenerator::plan);
}