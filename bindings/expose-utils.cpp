#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <boost/python.hpp>
#include "placo/trajectory/cubic_spline.h"
#include "placo/trajectory/cubic_spline_3d.h"
#include "module.h"
#include "placo/utils.h"
#include "expose-utils.hpp"

using namespace boost::python;

void exposeUtils()
{
  def("interpolate_frames", &placo::interpolate_frames);
  def("wrap_angle", &placo::wrap_angle);
  def("frame_yaw", &placo::frame_yaw);
  def("frame", &placo::frame);
  def("flatten_on_floor", &placo::flatten_on_floor);

  exposeStdVector<int>("vector_int");
  exposeStdVector<double>("vector_double");
  exposeStdVector<std::string>("vector_string");

  // exposeStdMap<std::string, double>("map_string_double");

  class_<std::map<std::string, double>>("map_string_double").def(map_indexing_suite<std::map<std::string, double>>());

  class_<placo::CubicSpline>("CubicSpline", init<optional<bool>>())
      .def("pos", &placo::CubicSpline::pos)
      .def("vel", &placo::CubicSpline::vel)
      .def("add_point", &placo::CubicSpline::add_point);

  class_<placo::CubicSpline3D>("CubicSpline3D")
      .def("pos", &placo::CubicSpline3D::pos)
      .def("vel", &placo::CubicSpline3D::vel)
      .def("add_point", &placo::CubicSpline3D::add_point);
}
