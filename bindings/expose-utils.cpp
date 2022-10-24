#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <boost/python.hpp>
#include "module.h"
#include "placo/utils.h"
#include "expose-utils.hpp"

using namespace boost::python;

void exposeUtils()
{
  def("interpolate_frames", &placo::interpolate_frames);
  def("frame_yaw", &placo::frame_yaw);
  def("frame", &placo::frame);
  def("flatten_on_floor", &placo::flatten_on_floor);
  
  exposeStdVector<int>("vector_int");
  exposeStdVector<double>("vector_double");
  exposeStdVector<std::string>("vector_string");
}