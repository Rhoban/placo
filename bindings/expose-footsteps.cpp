#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/footsteps/footsteps_planner.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeFootsteps() {
  enum_<FootstepsPlanner::Side>("FootstepsPlanner_Side")
      .value("left", FootstepsPlanner::Side::Left)
      .value("right", FootstepsPlanner::Side::Right);

  class_<FootstepsPlanner::Footstep>("Footstep", init<double, double>())
      .add_property("side", &FootstepsPlanner::Footstep::side)
      .add_property("frame", &FootstepsPlanner::Footstep::frame)
      .def("support_polygon", &FootstepsPlanner::Footstep::support_polygon);

  class_<FootstepsPlanner::Support>("Support").def(
      "support_polygon", &FootstepsPlanner::Support::support_polygon);

  class_<FootstepsPlanner>(
      "FootstepsPlanner",
      init<std::string, Eigen::Affine3d, Eigen::Affine3d, double>())
      .def("plan", &FootstepsPlanner::plan);

  // Exposing vector of footsteps
  exposeStdVector<FootstepsPlanner::Footstep>("Footsteps");
  // class_<std::vector<FootstepsPlanner::Footstep>>("Footsteps")
  //       .def(vector_indexing_suite<std::vector<FootstepsPlanner::Footstep>>());
}