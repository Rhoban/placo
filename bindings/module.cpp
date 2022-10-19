#include <eigenpy/eigenpy.hpp>
#include <boost/python.hpp>
#include "placo/utils.h"

BOOST_PYTHON_MODULE(placo) {
  using namespace boost::python;
  
  eigenpy::enableEigenPy();

  def("average_frames", &placo::average_frames);
  def("frame_yaw", &placo::frame_yaw);
}