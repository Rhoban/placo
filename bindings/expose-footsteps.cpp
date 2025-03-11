#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "registry.h"
#include "placo/humanoid/footsteps_planner.h"
#include "placo/humanoid/footsteps_planner_naive.h"
#include "placo/humanoid/footsteps_planner_repetitive.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigen-to-python.hpp>

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
      .def_readwrite("raw_frame", &FootstepsPlanner::Footstep::raw_frame)
      .add_property("dx", &FootstepsPlanner::Footstep::dx, &FootstepsPlanner::Footstep::dx)
      .add_property("dy", &FootstepsPlanner::Footstep::dy, &FootstepsPlanner::Footstep::dy)
      .add_property("foot_length", &FootstepsPlanner::Footstep::foot_length, &FootstepsPlanner::Footstep::foot_length)
      .add_property("foot_width", &FootstepsPlanner::Footstep::foot_width, &FootstepsPlanner::Footstep::foot_width)
      .def("support_polygon", &FootstepsPlanner::Footstep::support_polygon)
      .def("overlap", &FootstepsPlanner::Footstep::overlap)
      .def("polygon_contains", &FootstepsPlanner::Footstep::polygon_contains)
      .def("frame", &FootstepsPlanner::Footstep::frame)
      .staticmethod("polygon_contains");

  class__<FootstepsPlanner::Support>("Support", init<>())
      .def("support_polygon", &FootstepsPlanner::Support::support_polygon)
      .def("frame", &FootstepsPlanner::Support::frame)
      .def("footstep_frame", &FootstepsPlanner::Support::footstep_frame)
      .def("side", &FootstepsPlanner::Support::side)
      .def("is_both", &FootstepsPlanner::Support::is_both)
      .def(
          "set_start", +[](FootstepsPlanner::Support& support, bool b) { support.start = b; })
      .def(
          "set_end", +[](FootstepsPlanner::Support& support, bool b) { support.end = b; })
      .add_property("footsteps", &FootstepsPlanner::Support::footsteps)
      .add_property("t_start", &FootstepsPlanner::Support::t_start, &FootstepsPlanner::Support::t_start)
      .add_property("elapsed_ratio", &FootstepsPlanner::Support::elapsed_ratio, &FootstepsPlanner::Support::elapsed_ratio)
      .add_property("time_ratio", &FootstepsPlanner::Support::time_ratio, &FootstepsPlanner::Support::time_ratio)
      .add_property("start", &FootstepsPlanner::Support::start, &FootstepsPlanner::Support::start)
      .add_property("end", &FootstepsPlanner::Support::end, &FootstepsPlanner::Support::end)
      .add_property("replanned", &FootstepsPlanner::Support::replanned, &FootstepsPlanner::Support::replanned);

  class__<FootstepsPlanner, boost::noncopyable>("FootstepsPlanner", no_init)
      .def("make_supports", &FootstepsPlanner::make_supports)
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
