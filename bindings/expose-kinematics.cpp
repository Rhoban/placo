#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/control/kinematics_solver.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeKinematics() {

  class_<KinematicsSolver>("KinematicsSolver", init<MobileRobot &>())
      // Position and CoM task
      .def<void (KinematicsSolver::*)(std::string, Eigen::Vector3d, std::string,
                                      double)>(
          "add_position_task", &KinematicsSolver::add_position_task)
      .def<void (KinematicsSolver::*)(Eigen::Vector3d, std::string, double)>(
          "add_com_task", &KinematicsSolver::add_com_task)

      // Orientation task
      .def<void (KinematicsSolver::*)(std::string, Eigen::Matrix3d, std::string,
                                      double)>(
          "add_orientation_task", &KinematicsSolver::add_orientation_task)

      // Frame task
      .def<void (KinematicsSolver::*)(std::string, Eigen::Affine3d, std::string,
                                      double, double)>(
          "add_frame_task", &KinematicsSolver::add_frame_task)

      // Pose task
      .def<void (KinematicsSolver::*)(std::string, Eigen::Affine3d, std::string,
                                      double)>("add_pose_task",
                                               &KinematicsSolver::add_pose_task)

      // Regularization task
      .def("add_regularization_task",
           &KinematicsSolver::add_regularization_task)

      .def("solve", &KinematicsSolver::solve);

  // .def<Eigen::Affine3d (MobileRobot::*)(const std::string &)>(
}