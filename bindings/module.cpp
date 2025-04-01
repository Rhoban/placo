#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <boost/python.hpp>
#include <ostream>
#include <pinocchio/bindings/python/spatial/se3.hpp>

#include "module.h"

BOOST_PYTHON_MODULE(placo)
{
  using namespace boost::python;

  exposeEigen();
  exposeTools();
  exposeDynamics();
  exposeFootsteps();
  exposeProblem();
  exposeRobotWrapper();
  exposeParameters();
  exposeKinematics();
  exposeWalkPatternGenerator();
  exposeRegistry();
}