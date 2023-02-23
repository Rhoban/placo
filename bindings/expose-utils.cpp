#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <boost/python.hpp>
#include "rhoban_utils/spline/poly_spline.h"
#include "rhoban_utils/spline/poly_spline_3d.h"
#include "module.h"
#include "placo/utils.h"
#include "expose-utils.hpp"

using namespace boost::python;
using namespace rhoban_utils;

void exposeUtils()
{
  def("interpolate_frames", &placo::interpolate_frames);
  def("frame_yaw", &placo::frame_yaw);
  def("frame", &placo::frame);
  def("flatten_on_floor", &placo::flatten_on_floor);

  exposeStdVector<int>("vector_int");
  exposeStdVector<double>("vector_double");
  exposeStdVector<std::string>("vector_string");

  // exposeStdMap<std::string, double>("map_string_double");

  class_<std::map<std::string, double> >("map_string_double").def(map_indexing_suite<std::map<std::string, double> >());

  class_<PolySpline>("PolySpline")
      .def("get", &PolySpline::get)
      .def("getVel", &PolySpline::getVel)
      .def("addPoint", &PolySpline::addPoint);
  class_<PolySpline3D>("PolySpline3D")
      .def("get", &PolySpline3D::get)
      .def("getVel", &PolySpline3D::getVel)
      .def("addPoint", &PolySpline3D::addPoint);
}
