#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/footsteps/footsteps_planner_repetitive.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeFootsteps()
{
  enum_<HumanoidRobot::Side>("HumanoidRobot_Side")
      .value("left", HumanoidRobot::Side::Left)
      .value("right", HumanoidRobot::Side::Right)
      .value("both", HumanoidRobot::Side::Both);

  class_<FootstepsPlanner::Footstep>("Footstep", init<double, double>())
      .add_property("side", &FootstepsPlanner::Footstep::side)
      .add_property("frame", &FootstepsPlanner::Footstep::frame)
      .def("support_polygon", &FootstepsPlanner::Footstep::support_polygon);

  class_<FootstepsPlanner::Support>("Support")
      .def("support_polygon", &FootstepsPlanner::Support::support_polygon)
      .def("frame", &FootstepsPlanner::Support::frame)
      .add_property("footsteps", &FootstepsPlanner::Support::footsteps);

  class_<FootstepsPlanner, boost::noncopyable>("FootstepsPlanner", no_init)
      .def("make_supports", &FootstepsPlannerNaive::make_supports);

  class_<FootstepsPlannerNaive, bases<FootstepsPlanner>>("FootstepsPlannerNaive", init<HumanoidParameters&>())
      .def("plan", &FootstepsPlannerNaive::plan)
      .def("configure", &FootstepsPlannerNaive::configure);

  class_<FootstepsPlannerRepetitive, bases<FootstepsPlanner>>("FootstepsPlannerRepetitive", init<HumanoidParameters&>())
      .def("plan", &FootstepsPlannerRepetitive::plan)
      .def("configure", &FootstepsPlannerRepetitive::configure);

  // Exposing vector of footsteps
  exposeStdVector<FootstepsPlanner::Footstep>("Footsteps");
  exposeStdVector<FootstepsPlanner::Support>("Supports");
}