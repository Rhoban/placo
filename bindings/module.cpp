#include <pinocchio/fwd.hpp>

#include "placo/utils.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <ostream>
#include <pinocchio/bindings/python/spatial/se3.hpp>

#include "expose-eigen.h"
#include "expose-utils.h"

BOOST_PYTHON_MODULE(placo) {
  using namespace boost::python;

  eigenpy::enableEigenPy();
  exposeAffine3d();
  exposeUtils();
}