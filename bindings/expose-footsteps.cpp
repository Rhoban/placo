#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "registry.h"
#include "placo/humanoid/footsteps_planner.h"
#include "placo/humanoid/footsteps_planner_naive.h"
#include "placo/humanoid/footsteps_planner_repetitive.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo::humanoid;
using namespace placo;

void exposeFootsteps()
{
  enum_<HumanoidRobot::Side>("HumanoidRobot_Side")
      .value("left", HumanoidRobot::Side::Left)
      .value("right", HumanoidRobot::Side::Right);

  class__<FootstepsPlanner::Footstep>("Footstep", init<double, double>())
      .def("support_polygon", &FootstepsPlanner::Footstep::support_polygon)
      .add_property("side", &FootstepsPlanner::Footstep::side, &FootstepsPlanner::Footstep::side)
      .add_property(
          "frame", +[](const FootstepsPlanner::Footstep& footstep) { return footstep.frame; },
          &FootstepsPlanner::Footstep::frame)
      .add_property("foot_length", &FootstepsPlanner::Footstep::foot_length, &FootstepsPlanner::Footstep::foot_length)
      .add_property("foot_width", &FootstepsPlanner::Footstep::foot_width, &FootstepsPlanner::Footstep::foot_width)
      .def("support_polygon", &FootstepsPlanner::Footstep::support_polygon)
      .def("overlap", &FootstepsPlanner::Footstep::overlap)
      .def("polygon_contains", &FootstepsPlanner::Footstep::polygon_contains)
      .staticmethod("polygon_contains")
      .add_property("kick", &FootstepsPlanner::Footstep::kick, &FootstepsPlanner::Footstep::kick);

  class__<FootstepsPlanner::Support>("Support")
      .def("support_polygon", &FootstepsPlanner::Support::support_polygon)
      .def("frame", &FootstepsPlanner::Support::frame)
      .def("footstep_frame", &FootstepsPlanner::Support::footstep_frame)
      .def("side", &FootstepsPlanner::Support::side)
      .def("is_both", &FootstepsPlanner::Support::is_both)
      .def(
          "set_start", +[](FootstepsPlanner::Support& support, bool b) { support.start = b; })
      .def(
          "set_end", +[](FootstepsPlanner::Support& support, bool b) { support.end = b; })
      .def("kick", &FootstepsPlanner::Support::kick)
      .add_property("footsteps", &FootstepsPlanner::Support::footsteps)
      .add_property("start", &FootstepsPlanner::Support::start, &FootstepsPlanner::Support::start)
      .add_property("end", &FootstepsPlanner::Support::end, &FootstepsPlanner::Support::end);

  class__<FootstepsPlanner, boost::noncopyable>("FootstepsPlanner", no_init)
      .def("make_supports", &FootstepsPlanner::make_supports)
      .def("add_first_support", &FootstepsPlanner::add_first_support)
      .def("opposite_footstep", &FootstepsPlanner::opposite_footstep);

  class__<FootstepsPlannerNaive, bases<FootstepsPlanner>>("FootstepsPlannerNaive", init<HumanoidParameters&>())
      .def("plan", &FootstepsPlannerNaive::plan)
      .def("configure", &FootstepsPlannerNaive::configure);

  class__<FootstepsPlannerRepetitive, bases<FootstepsPlanner>>("FootstepsPlannerRepetitive",
                                                               init<HumanoidParameters&>())
      .def("plan", &FootstepsPlannerRepetitive::plan)
      .def("configure", &FootstepsPlannerRepetitive::configure);

  // Exposing vector of footsteps
  exposeStdVector<FootstepsPlanner::Footstep>("Footsteps");
  exposeStdVector<FootstepsPlanner::Support>("Supports");
}
