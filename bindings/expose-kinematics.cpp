#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "doxystub.h"
#include "placo/kinematics/kinematics_solver.h"
#include <boost/python/return_internal_reference.hpp>
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigen-to-python.hpp>

using namespace boost::python;
using namespace placo;
using namespace placo::kinematics;
using namespace placo::model;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(frametask_configure_overloads, configure, 2, 4);

void exposeKinematics()
{
  class_<KinematicsSolver> solver_class =
      class__<KinematicsSolver>("KinematicsSolver", init<RobotWrapper&>())
          .add_property("problem", &KinematicsSolver::problem)
          .add_property("dt", &KinematicsSolver::dt, &KinematicsSolver::dt)
          .add_property("N", &KinematicsSolver::N)
          .add_property("scale", &KinematicsSolver::scale)
          .add_property(
              "robot", +[](const KinematicsSolver& solver) { return solver.robot; })

          // Position and CoM task
          .def<PositionTask& (KinematicsSolver::*)(std::string, Eigen::Vector3d)>(
              "add_position_task", &KinematicsSolver::add_position_task, return_internal_reference<>())
          .def<RelativePositionTask& (KinematicsSolver::*)(std::string, std::string, Eigen::Vector3d)>(
              "add_relative_position_task", &KinematicsSolver::add_relative_position_task,
              return_internal_reference<>())
          .def<CoMTask& (KinematicsSolver::*)(Eigen::Vector3d)>("add_com_task", &KinematicsSolver::add_com_task,
                                                                return_internal_reference<>())

          // Orientation task
          .def<OrientationTask& (KinematicsSolver::*)(std::string, Eigen::Matrix3d)>(
              "add_orientation_task", &KinematicsSolver::add_orientation_task, return_internal_reference<>())
          .def<RelativeOrientationTask& (KinematicsSolver::*)(std::string, std::string, Eigen::Matrix3d)>(
              "add_relative_orientation_task", &KinematicsSolver::add_relative_orientation_task,
              return_internal_reference<>())

          // Frame task
          .def<FrameTask (KinematicsSolver::*)(std::string, Eigen::Affine3d)>("add_frame_task",
                                                                              &KinematicsSolver::add_frame_task)
          .def<RelativeFrameTask (KinematicsSolver::*)(std::string, std::string, Eigen::Affine3d)>(
              "add_relative_frame_task", &KinematicsSolver::add_relative_frame_task)

          // Axis align task
          .def<AxisAlignTask& (KinematicsSolver::*)(std::string, Eigen::Vector3d, Eigen::Vector3d)>(
              "add_axisalign_task", &KinematicsSolver::add_axisalign_task, return_internal_reference<>())

          // Joint task
          .def<JointsTask& (KinematicsSolver::*)(void)>("add_joints_task", &KinematicsSolver::add_joints_task,
                                                        return_internal_reference<>())

          // Gear task
          .def<GearTask& (KinematicsSolver::*)(void)>("add_gear_task", &KinematicsSolver::add_gear_task,
                                                      return_internal_reference<>())

          // Wheel task
          .def<WheelTask& (KinematicsSolver::*)(std::string, double, bool)>(
              "add_wheel_task", &KinematicsSolver::add_wheel_task, return_internal_reference<>())

          // Distance task
          .def<DistanceTask& (KinematicsSolver::*)(std::string, std::string, double)>(
              "add_distance_task", &KinematicsSolver::add_distance_task, return_internal_reference<>())

          // Centroidal Momentum Task
          .def<CentroidalMomentumTask& (KinematicsSolver::*)(Eigen::Vector3d)>(
              "add_centroidal_momentum_task", &KinematicsSolver::add_centroidal_momentum_task,
              return_internal_reference<>())

          // Regularization task
          .def("add_regularization_task", &KinematicsSolver::add_regularization_task, return_internal_reference<>())

          // Manipulability task
          .def<ManipulabilityTask& (KinematicsSolver::*)(std::string, std::string, double)>(
              "add_manipulability_task", &KinematicsSolver::add_manipulability_task, return_internal_reference<>())

          // Kinetic energy regularization task
          .def("add_kinetic_energy_regularization_task", &KinematicsSolver::add_kinetic_energy_regularization_task,
               return_internal_reference<>())

          // Avoid self collisions constraint
          .def("add_avoid_self_collisions_constraint", &KinematicsSolver::add_avoid_self_collisions_constraint,
               return_internal_reference<>())

          // CoM polygon
          .def("add_com_polygon_constraint", &KinematicsSolver::add_com_polygon_constraint,
               return_internal_reference<>())

          // Joint-space half-spaces
          .def("add_joint_space_half_spaces_constraint", &KinematicsSolver::add_joint_space_half_spaces_constraint,
               return_internal_reference<>())

          // Cone constraint
          .def<ConeConstraint& (KinematicsSolver::*)(std::string, std::string, double)>(
              "add_cone_constraint", &KinematicsSolver::add_cone_constraint, return_internal_reference<>())

          // Yaw constraint
          .def<YawConstraint& (KinematicsSolver::*)(std::string, std::string, double)>(
              "add_yaw_constraint", &KinematicsSolver::add_yaw_constraint, return_internal_reference<>())

          // Distance constraint
          .def<DistanceConstraint& (KinematicsSolver::*)(std::string, std::string, double)>(
              "add_distance_constraint", &KinematicsSolver::add_distance_constraint, return_internal_reference<>())

          // Masking/unmasking DoFs
          .def("mask_dof", &KinematicsSolver::mask_dof)
          .def("unmask_dof", &KinematicsSolver::unmask_dof)
          .def("mask_fbase", &KinematicsSolver::mask_fbase)

          // Clearing the tasks
          .def("clear", &KinematicsSolver::clear)
          .def("dump_status", &KinematicsSolver::dump_status)

          .def("tasks_count", &KinematicsSolver::tasks_count)

          .def("enable_joint_limits", &KinematicsSolver::enable_joint_limits)
          .def("enable_velocity_limits", &KinematicsSolver::enable_velocity_limits)
          .def<void (KinematicsSolver::*)(Task&)>("add_task", &KinematicsSolver::add_task)
          .def<void (KinematicsSolver::*)(Constraint&)>("add_constraint", &KinematicsSolver::add_constraint)
          .def<void (KinematicsSolver::*)(Task&)>("remove_task", &KinematicsSolver::remove_task)
          .def<void (KinematicsSolver::*)(FrameTask&)>("remove_task", &KinematicsSolver::remove_task)
          .def("remove_constraint", &KinematicsSolver::remove_constraint)
          .def("solve", &KinematicsSolver::solve);

  class__<Task, bases<tools::Prioritized>, boost::noncopyable>("Task", no_init)
      .def_readonly("A", &Task::A)
      .def_readonly("b", &Task::b)
      .def("error", &Task::error)
      .def("error_norm", &Task::error_norm)
      .def("update", &Task::update);

  class__<PositionTask, bases<Task>>("PositionTask", init<RobotWrapper::FrameIndex, Eigen::Vector3d>())
      .add_property("frame_index", &PositionTask::frame_index)
      .add_property(
          "target_world", +[](const PositionTask& task) { return task.target_world; }, &PositionTask::target_world)
      .add_property("mask", &PositionTask::mask, &PositionTask::mask);

  class__<RelativePositionTask, bases<Task>>(
      "RelativePositionTask", init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Vector3d>())
      .add_property("frame_a", &RelativePositionTask::frame_a)
      .add_property("frame_b", &RelativePositionTask::frame_b)
      .def_readwrite("target", &RelativePositionTask::target)
      .add_property("mask", &RelativePositionTask::mask, &RelativePositionTask::mask);

  class__<CoMTask, bases<Task>>("CoMTask", init<Eigen::Vector3d>())
      .add_property(
          "target_world", +[](const CoMTask& task) { return task.target_world; }, &CoMTask::target_world)
      .add_property("mask", &CoMTask::mask, &CoMTask::mask);

  class__<OrientationTask, bases<Task>>("OrientationTask", init<RobotWrapper::FrameIndex, Eigen::Matrix3d>())
      .add_property("frame_index", &OrientationTask::frame_index)
      .def_readwrite("R_world_frame", &OrientationTask::R_world_frame)
      .add_property("mask", &OrientationTask::mask, &OrientationTask::mask);

  class__<RelativeOrientationTask, bases<Task>>(
      "RelativeOrientationTask", init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Matrix3d>())
      .add_property("frame_a", &RelativeOrientationTask::frame_a)
      .add_property("frame_b", &RelativeOrientationTask::frame_b)
      .def_readwrite("R_a_b", &RelativeOrientationTask::R_a_b)
      .add_property("mask", &RelativeOrientationTask::mask, &RelativeOrientationTask::mask);

  class__<FrameTask>("FrameTask", init<>())
      .def(
          "position", +[](const FrameTask& task) -> PositionTask& { return *task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const FrameTask& task) -> OrientationTask& { return *task.orientation; },
          return_internal_reference<>())
      .def("configure", &FrameTask::configure, frametask_configure_overloads())
      .add_property("T_world_frame", &FrameTask::get_T_world_frame, &FrameTask::set_T_world_frame);

  class__<RelativeFrameTask>("RelativeFrameTask", init<RelativePositionTask&, RelativeOrientationTask&>())
      .def(
          "position", +[](const RelativeFrameTask& task) -> RelativePositionTask& { return task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const RelativeFrameTask& task) -> RelativeOrientationTask& { return task.orientation; },
          return_internal_reference<>())
      .def("configure", &RelativeFrameTask::configure, frametask_configure_overloads())
      .add_property("T_a_b", &RelativeFrameTask::get_T_a_b, &RelativeFrameTask::set_T_a_b);

  class__<AxisAlignTask, bases<Task>>("AxisAlignTask",
                                      init<RobotWrapper::FrameIndex, Eigen::Vector3d, Eigen::Vector3d>())
      .add_property("frame_index", &AxisAlignTask::frame_index)
      .add_property(
          "axis_frame", +[](const AxisAlignTask& task) { return task.axis_frame; }, &AxisAlignTask::axis_frame)
      .def_readwrite("targetAxis_world", &AxisAlignTask::targetAxis_world);

  class__<JointsTask, bases<Task>>("JointsTask", init<>())
      .def("set_joint", &JointsTask::set_joint)
      .def("get_joint", &JointsTask::get_joint)
      .def(
          "set_joints", +[](JointsTask& task, boost::python::dict& py_dict) {
            update_map<std::string, double>(task.joints, py_dict);
          });

  class__<GearTask, bases<Task>>("GearTask", init<>())
      .def("set_gear", &GearTask::set_gear)
      .def("add_gear", &GearTask::add_gear);

  class__<WheelTask, bases<Task>>("WheelTask", init<std::string, double, bool>())
      .add_property("joint", &WheelTask::joint)
      .add_property("radius", &WheelTask::radius)
      .add_property("omniwheel", &WheelTask::omniwheel)
      .def_readwrite("T_world_surface", &WheelTask::T_world_surface);

  class__<DistanceTask, bases<Task>>("DistanceTask", init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, double>())
      .add_property("frame_a", &DistanceTask::frame_a)
      .add_property("frame_b", &DistanceTask::frame_b)
      .add_property("distance", &DistanceTask::distance, &DistanceTask::distance);

  class__<CentroidalMomentumTask, bases<Task>>("CentroidalMomentumTask", init<Eigen::Vector3d>())
      .add_property("mask", &CentroidalMomentumTask::mask)
      .add_property("L_world", &CentroidalMomentumTask::L_world, &CentroidalMomentumTask::L_world);

  class__<RegularizationTask, bases<Task>>("RegularizationTask");

  class__<ManipulabilityTask, bases<Task>>("ManipulabilityTask",
                                           init<RobotWrapper::FrameIndex, ManipulabilityTask::Type, double>())
      .def_readwrite("lambda_", &ManipulabilityTask::lambda)
      .def_readwrite("minimize", &ManipulabilityTask::minimize)
      .def_readonly("manipulability", &ManipulabilityTask::manipulability);

  class__<KineticEnergyRegularizationTask, bases<RegularizationTask>>("KineticEnergyRegularizationTask");

  class__<Constraint, bases<tools::Prioritized>, boost::noncopyable>("KinematicsConstraint", no_init);

  class__<AvoidSelfCollisionsConstraint, bases<Constraint>>("AvoidSelfCollisionsKinematicsConstraint", init<>())
      .def_readwrite("self_collisions_margin", &AvoidSelfCollisionsConstraint::self_collisions_margin)
      .def_readwrite("self_collisions_trigger", &AvoidSelfCollisionsConstraint::self_collisions_trigger);

  class__<CoMPolygonConstraint, bases<Constraint>>("CoMPolygonConstraint", init<std::vector<Eigen::Vector2d>, double>())
      .def_readwrite("polygon", &CoMPolygonConstraint::polygon)
      .def_readwrite("dcm", &CoMPolygonConstraint::dcm)
      .def_readwrite("omega", &CoMPolygonConstraint::omega)
      .def_readwrite("margin", &CoMPolygonConstraint::margin);

  class__<JointSpaceHalfSpacesConstraint, bases<Constraint>>("JointSpaceHalfSpacesConstraint",
                                                             init<Eigen::MatrixXd, Eigen::VectorXd>())
      .def_readwrite("A", &JointSpaceHalfSpacesConstraint::A)
      .def_readwrite("b", &JointSpaceHalfSpacesConstraint::b);

  class__<ConeConstraint, bases<Constraint>>(
      "ConeConstraint", init<model::RobotWrapper::FrameIndex, model::RobotWrapper::FrameIndex, double>())
      .def_readwrite("angle_max", &ConeConstraint::angle_max)
      .def_readwrite("N", &ConeConstraint::N)
      .def_readwrite("range", &ConeConstraint::range);

  class__<YawConstraint, bases<Constraint>>(
      "YawConstraint", init<model::RobotWrapper::FrameIndex, model::RobotWrapper::FrameIndex, double>())
      .def_readwrite("angle_max", &YawConstraint::angle_max);

  class__<DistanceConstraint, bases<Constraint>>(
      "DistanceConstraint", init<model::RobotWrapper::FrameIndex, model::RobotWrapper::FrameIndex, double>())
      .def_readwrite("distance_max", &DistanceConstraint::distance_max);
}