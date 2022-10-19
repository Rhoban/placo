#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <boost/python.hpp>
#include "expose-utils.h"
#include "placo/utils.h"

using namespace boost::python;

void exposeUtils()
{
  def("interpolate_frames", &placo::interpolate_frames);
  def("frame_yaw", &placo::frame_yaw);
}