#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/model/robot_wrapper.h"
#include "placo/dynamics/dynamics_solver.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace placo;
using namespace placo::dynamics;

static class_<DynamicsSolver>* solver_class_ptr = nullptr;

template <typename T>
void registerTaskMethods(class_<T>& class__)
{
  class__.add_property("name", &T::name)
      .add_property("solver", &T::solver)
      .add_property("weight", &T::weight)
      .add_property(
          "priority", +[](T& task) { return task.priority_name(); })
      .add_property("A", &T::A)
      .add_property("b", &T::b)
      .add_property("kp", &T::kp, &T::kp)
      .def("error", &T::error)
      .def("error_norm", &T::error_norm)
      .def("update", &T::update)
      .def("set_priority", &T::set_priority)
      .def("set_weight", &T::set_weight)
      .def("set_name", &T::set_name)
      .def(
          "configure", +[](T& task, std::string name, std::string priority, double weight) {
            task.configure(name, priority, weight);
          });

  solver_class_ptr->def(
      "remove_task", +[](DynamicsSolver& solver, T& task) { solver.remove_task(&task); });
}

void exposeContacts()
{
  class_<DynamicsSolver::Result>("DynamicsSolverResult")
      .add_property("success", &DynamicsSolver::Result::success)
      .add_property(
          "tau", +[](const DynamicsSolver::Result& result) { return result.tau; })
      .add_property(
          "qdd", +[](const DynamicsSolver::Result& result) { return result.qdd; })
      .def(
          "tau_dict", +[](const DynamicsSolver::Result& result, RobotWrapper& robot) {
            boost::python::dict dict;

            for (auto& dof : robot.actuated_joint_names())
            {
              dict[dof] = result.tau[robot.get_joint_v_offset(dof)];
            }

            return dict;
          });

  class_<PointContact>("DynamicsSolverPointContact", init<PositionTask&, bool>())
      .def_readwrite("mu", &PointContact::mu)
      .def_readwrite("weight_forces", &PointContact::weight_forces)
      .add_property(
          "wrench", +[](PointContact& contact) { return contact.variable->value; })
      .def_readwrite("unilateral", &PointContact::unilateral);

  class_<PlanarContact>("DynamicsSolverPlanarContact", init<FrameTask&, bool>())
      .def_readwrite("mu", &PlanarContact::mu)
      .def_readwrite("weight_forces", &PlanarContact::weight_forces)
      .def_readwrite("weight_moments", &PlanarContact::weight_moments)
      .add_property(
          "wrench", +[](PlanarContact& contact) { return contact.variable->value; })
      .def_readwrite("unilateral", &PlanarContact::unilateral)
      .def_readwrite("length", &PlanarContact::length)
      .def_readwrite("width", &PlanarContact::width)
      .def("zmp", &PlanarContact::zmp);

  class_<RelativePointContact>("DynamicsSolverRelativePointContact", init<RelativePositionTask&>())
      .def_readwrite("weight_forces", &RelativePointContact::weight_forces)
      .add_property(
          "wrench", +[](RelativePointContact& contact) { return contact.variable->value; });

  class_<ExternalWrenchContact>("DynamicsSolverExternalWrenchContact", init<RobotWrapper::FrameIndex>())
      .def_readwrite("w_ext", &ExternalWrenchContact::w_ext);

  class_<PuppetContact>("DynamicsSolverPuppetContact", init<>());

  class_<DynamicsSolver> solver_class =
      class_<DynamicsSolver>("DynamicsSolver", init<RobotWrapper&>())
          .def_readwrite("dt", &DynamicsSolver::dt)
          .def("add_point_contact", &DynamicsSolver::add_point_contact, return_internal_reference<>())
          .def("add_unilateral_point_contact", &DynamicsSolver::add_unilateral_point_contact,
               return_internal_reference<>())
          .def("add_relative_point_contact", &DynamicsSolver::add_relative_point_contact, return_internal_reference<>())
          .def("add_planar_contact", &DynamicsSolver::add_planar_contact, return_internal_reference<>())
          .def("add_fixed_contact", &DynamicsSolver::add_fixed_contact, return_internal_reference<>())
          .def<ExternalWrenchContact& (DynamicsSolver::*)(std::string)>("add_external_wrench_contact",
                                                                        &DynamicsSolver::add_external_wrench_contact,
                                                                        return_internal_reference<>())
          .def("add_puppet_contact", &DynamicsSolver::add_puppet_contact, return_internal_reference<>())
          .def("set_passive", &DynamicsSolver::set_passive)
          .def("enable_velocity_limits", &DynamicsSolver::enable_velocity_limits)
          .def("enable_joint_limits", &DynamicsSolver::enable_joint_limits)
          .def("enable_torque_limits", &DynamicsSolver::enable_torque_limits)
          .def("enable_self_collision_avoidance", &DynamicsSolver::enable_self_collision_avoidance)
          .def("configure_self_collision_avoidance", &DynamicsSolver::configure_self_collision_avoidance)
          .def("set_static", &DynamicsSolver::set_static)
          .def("solve", &DynamicsSolver::solve)
          .def<PositionTask& (DynamicsSolver::*)(std::string, Eigen::Vector3d)>(
              "add_position_task", &DynamicsSolver::add_position_task, return_internal_reference<>())
          .def<RelativePositionTask& (DynamicsSolver::*)(std::string, std::string, Eigen::Vector3d)>(
              "add_relative_position_task", &DynamicsSolver::add_relative_position_task, return_internal_reference<>())
          .def<OrientationTask& (DynamicsSolver::*)(std::string, Eigen::Matrix3d)>(
              "add_orientation_task", &DynamicsSolver::add_orientation_task, return_internal_reference<>())
          .def<JointsTask& (DynamicsSolver::*)()>("add_joints_task", &DynamicsSolver::add_joints_task,
                                                  return_internal_reference<>())
          .def<CoMTask& (DynamicsSolver::*)(Eigen::Vector3d)>("add_com_task", &DynamicsSolver::add_com_task,
                                                              return_internal_reference<>())
          .def<FrameTask (DynamicsSolver::*)(std::string, Eigen::Affine3d)>("add_frame_task",
                                                                            &DynamicsSolver::add_frame_task);

  solver_class_ptr = &solver_class;

  registerTaskMethods(
      class_<PositionTask>("DynamicsPositionTask", init<RobotWrapper::FrameIndex, Eigen::Vector3d>())
          .add_property(
              "target_world", +[](const PositionTask& task) { return task.target_world; }, &PositionTask::target_world)
          .add_property(
              "dtarget_world", +[](const PositionTask& task) { return task.dtarget_world; },
              &PositionTask::dtarget_world)
          .add_property("mask", &PositionTask::mask, &PositionTask::mask));

  registerTaskMethods(
      class_<CoMTask>("DynamicsCoMTask", init<Eigen::Vector3d>())
          .add_property(
              "target_world", +[](const CoMTask& task) { return task.target_world; }, &CoMTask::target_world)
          .add_property(
              "dtarget_world", +[](const CoMTask& task) { return task.dtarget_world; }, &CoMTask::dtarget_world)
          .add_property("mask", &CoMTask::mask, &CoMTask::mask));

  registerTaskMethods(
      class_<RelativePositionTask>("DynamicsRelativePositionTask",
                                   init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Vector3d>())
          .add_property(
              "target", +[](const RelativePositionTask& task) { return task.target; }, &RelativePositionTask::target)
          .add_property(
              "dtarget", +[](const RelativePositionTask& task) { return task.dtarget; }, &RelativePositionTask::dtarget)
          .add_property("mask", &RelativePositionTask::mask, &RelativePositionTask::mask));

  registerTaskMethods(
      class_<OrientationTask>("DynamicsOrientationTask", init<RobotWrapper::FrameIndex, Eigen::Matrix3d>())
          .add_property(
              "R_world_frame", +[](const OrientationTask& task) { return task.R_world_frame; },
              &OrientationTask::R_world_frame)
          .add_property(
              "omega_world", +[](const OrientationTask& task) { return task.omega_world; },
              &OrientationTask::omega_world)
          .add_property("mask", &OrientationTask::mask, &OrientationTask::mask));

  class_<FrameTask>("DynamicsFrameTask", init<>())
      .def(
          "position", +[](const FrameTask& task) -> PositionTask& { return *task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const FrameTask& task) -> OrientationTask& { return *task.orientation; },
          return_internal_reference<>())
      .def("configure", &FrameTask::configure)
      .add_property("T_world_frame", &FrameTask::get_T_world_frame, &FrameTask::set_T_world_frame);

  registerTaskMethods(class_<JointsTask>("DynamicsJointsTask", init<>())
                          .def("set_joint", &JointsTask::set_joint)
                          .def(
                              "set_joints",
                              +[](JointsTask& task, boost::python::dict& py_dict) {
                                update_map<std::string, double>(task.joints, py_dict);
                              })
                          .def(
                              "set_joints_velocities", +[](JointsTask& task, boost::python::dict& py_dict) {
                                update_map<std::string, double>(task.djoints, py_dict);
                              }));
}