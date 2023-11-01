#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <boost/python.hpp>
#include "placo/trajectory/cubic_spline.h"
#include "placo/trajectory/cubic_spline_3d.h"
#include "placo/tools/axises_mask.h"
#include "module.h"
#include "registry.h"
#include "expose-utils.hpp"

using namespace boost::python;

using namespace placo::trajectory;

void exposeTrajectory()
{
  class__<CubicSpline>("CubicSpline", init<optional<bool>>())
      .def("pos", &CubicSpline::pos)
      .def("vel", &CubicSpline::vel)
      .def("add_point", &CubicSpline::add_point);

  class__<CubicSpline3D>("CubicSpline3D")
      .def("pos", &CubicSpline3D::pos)
      .def("vel", &CubicSpline3D::vel)
      .def("add_point", &CubicSpline3D::add_point);
}
