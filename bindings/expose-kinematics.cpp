#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "registry.h"
#include "placo/kinematics/kinematics_solver.h"
#include <boost/python/return_internal_reference.hpp>
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;
using namespace placo::kinematics;

void exposeKinematics()
{
  class_<KinematicsSolver> solver_class =
      class__<KinematicsSolver>("KinematicsSolver", init<RobotWrapper&>())
          .add_property("problem", &KinematicsSolver::problem)
          .add_property("noise", &KinematicsSolver::noise, &KinematicsSolver::noise)
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

          // Joint task
          .def<JointsTask& (KinematicsSolver::*)(void)>("add_joints_task", &KinematicsSolver::add_joints_task,
                                                        return_internal_reference<>())

          // Gear task
          .def<GearTask& (KinematicsSolver::*)(void)>("add_gear_task", &KinematicsSolver::add_gear_task,
                                                      return_internal_reference<>())

          // Distance task
          .def<DistanceTask& (KinematicsSolver::*)(std::string, std::string, double)>(
              "add_distance_task", &KinematicsSolver::add_distance_task, return_internal_reference<>())

          // Centroidal Momentum Task
          .def<CentroidalMomentumTask& (KinematicsSolver::*)(Eigen::Vector3d)>(
              "add_centroidal_momentum_task", &KinematicsSolver::add_centroidal_momentum_task,
              return_internal_reference<>())

          // Regularization task
          .def("add_regularization_task", &KinematicsSolver::add_regularization_task, return_internal_reference<>())

          // Avoid self collisions constraint
          .def("add_avoid_self_collisions_constraint", &KinematicsSolver::add_avoid_self_collisions_constraint,
               return_internal_reference<>())

          // CoM polygon
          .def("add_com_polygon_constraint", &KinematicsSolver::add_com_polygon_constraint,
               return_internal_reference<>())

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
      .add_property("A", &Task::A)
      .add_property("b", &Task::b)
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
      .add_property(
          "target", +[](const RelativePositionTask& task) { return task.target; }, &RelativePositionTask::target)
      .add_property("mask", &RelativePositionTask::mask, &RelativePositionTask::mask);

  class__<CoMTask, bases<Task>>("CoMTask", init<Eigen::Vector3d>())
      .add_property(
          "target_world", +[](const CoMTask& task) { return task.target_world; }, &CoMTask::target_world)
      .add_property("mask", &CoMTask::mask, &CoMTask::mask);

  class__<OrientationTask, bases<Task>>("OrientationTask", init<RobotWrapper::FrameIndex, Eigen::Matrix3d>())
      .add_property("frame_index", &OrientationTask::frame_index)
      .add_property(
          "R_world_frame", +[](const OrientationTask& task) { return task.R_world_frame; },
          &OrientationTask::R_world_frame)
      .add_property("mask", &OrientationTask::mask, &OrientationTask::mask);

  class__<RelativeOrientationTask, bases<Task>>(
      "RelativeOrientationTask", init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Matrix3d>())
      .add_property("frame_a", &RelativeOrientationTask::frame_a)
      .add_property("frame_b", &RelativeOrientationTask::frame_b)
      .add_property(
          "R_a_b", +[](const RelativeOrientationTask& task) { return task.R_a_b; }, &RelativeOrientationTask::R_a_b)
      .add_property("mask", &RelativeOrientationTask::mask, &RelativeOrientationTask::mask);

  class__<FrameTask>("FrameTask", init<>())
      .def(
          "position", +[](const FrameTask& task) -> PositionTask& { return *task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const FrameTask& task) -> OrientationTask& { return *task.orientation; },
          return_internal_reference<>())
      .def("configure", &FrameTask::configure)
      .add_property("T_world_frame", &FrameTask::get_T_world_frame, &FrameTask::set_T_world_frame);

  class__<RelativeFrameTask>("RelativeFrameTask", init<RelativePositionTask&, RelativeOrientationTask&>())
      .def(
          "position", +[](const RelativeFrameTask& task) -> RelativePositionTask& { return task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const RelativeFrameTask& task) -> RelativeOrientationTask& { return task.orientation; },
          return_internal_reference<>())
      .def("configure", &RelativeFrameTask::configure)
      .add_property("T_a_b", &RelativeFrameTask::get_T_a_b, &RelativeFrameTask::set_T_a_b);

  class__<JointsTask, bases<Task>>("JointsTask", init<>())
      .def("set_joint", &JointsTask::set_joint)
      .def(
          "set_joints", +[](JointsTask& task, boost::python::dict& py_dict) {
            update_map<std::string, double>(task.joints, py_dict);
          });

  class__<GearTask, bases<Task>>("GearTask", init<>()).def("set_gear", &GearTask::set_gear);

  class__<DistanceTask, bases<Task>>("DistanceTask", init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, double>())
      .add_property("frame_a", &DistanceTask::frame_a)
      .add_property("frame_b", &DistanceTask::frame_b)
      .add_property("distance", &DistanceTask::distance, &DistanceTask::distance);

  class__<CentroidalMomentumTask, bases<Task>>("CentroidalMomentumTask", init<Eigen::Vector3d>())
      .add_property("mask", &CentroidalMomentumTask::mask)
      .add_property("L_world", &CentroidalMomentumTask::L_world, &CentroidalMomentumTask::L_world);

  class__<RegularizationTask, bases<Task>>("RegularizationTask");

  class__<Constraint, bases<tools::Prioritized>, boost::noncopyable>("KinematicsConstraint", no_init);

  class__<AvoidSelfCollisionsConstraint, bases<Constraint>>("KinematicsAvoidSelfCollisionsConstraint", init<>())
      .def_readwrite("self_collisions_margin", &AvoidSelfCollisionsConstraint::self_collisions_margin)
      .def_readwrite("self_collisions_trigger", &AvoidSelfCollisionsConstraint::self_collisions_trigger);

  class__<CoMPolygonConstraint, bases<Constraint>>("CoMPolygonConstraint", init<std::vector<Eigen::Vector2d>, double>())
      .def_readwrite("polygon", &CoMPolygonConstraint::polygon)
      .def_readwrite("margin", &CoMPolygonConstraint::margin);
}