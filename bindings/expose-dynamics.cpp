#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/model/robot_wrapper.h"
#include "placo/dynamics/dynamics_solver.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace placo;
using namespace placo::dynamics;

void exposeDynamics()
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

  class_<Contact, boost::noncopyable>("Contact", no_init)
      .def_readwrite("mu", &Contact::mu)
      .def_readwrite("weight_forces", &Contact::weight_forces)
      .def_readwrite("weight_moments", &Contact::weight_moments)
      .add_property(
          "wrench", +[](Contact& contact) { return contact.wrench; });

  class_<PointContact, bases<Contact>>("PointContact", init<PositionTask&, bool>())
      .def(
          "position_task", +[](PointContact& contact) -> PositionTask& { return *contact.position_task; },
          return_internal_reference<>())
      .def_readwrite("unilateral", &PointContact::unilateral);

  class_<PlanarContact, bases<Contact>>("PlanarContact", init<FrameTask&, bool>())
      .def(
          "position_task", +[](PlanarContact& contact) -> PositionTask& { return *contact.position_task; },
          return_internal_reference<>())
      .def(
          "orientation_task", +[](PlanarContact& contact) -> OrientationTask& { return *contact.orientation_task; },
          return_internal_reference<>())
      .def_readwrite("unilateral", &PlanarContact::unilateral)
      .def_readwrite("length", &PlanarContact::length)
      .def_readwrite("width", &PlanarContact::width)
      .def("zmp", &PlanarContact::zmp);

  class_<RelativePointContact, bases<Contact>>("RelativePointContact", init<RelativePositionTask&>());

  class_<ExternalWrenchContact, bases<Contact>>("ExternalWrenchContact", init<RobotWrapper::FrameIndex>())
      .add_property("frame_index", &ExternalWrenchContact::frame_index)
      .add_property(
          "w_ext", +[](ExternalWrenchContact& contact) { return contact.w_ext; }, &ExternalWrenchContact::w_ext);

  class_<PuppetContact, bases<Contact>>("PuppetContact", init<>());

  class_<TaskContact, bases<Contact>>("TaskContact", init<Task&>());

  class_<DynamicsSolver> solver_class =
      class_<DynamicsSolver>("DynamicsSolver", init<RobotWrapper&>())
          .def_readwrite("friction", &DynamicsSolver::friction)
          .def_readwrite("dt", &DynamicsSolver::dt)
          .def_readwrite("qdd_safe", &DynamicsSolver::qdd_safe)
          .def_readwrite("xdd_safe", &DynamicsSolver::xdd_safe)
          .def_readwrite("optimize_contact_forces", &DynamicsSolver::optimize_contact_forces)
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
          .def("add_task_contact", &DynamicsSolver::add_task_contact, return_internal_reference<>())
          .def("set_passive", &DynamicsSolver::set_passive)
          .def("enable_velocity_limits", &DynamicsSolver::enable_velocity_limits)
          .def("enable_velocity_vs_torque_limits", &DynamicsSolver::enable_velocity_vs_torque_limits)
          .def("enable_joint_limits", &DynamicsSolver::enable_joint_limits)
          .def("enable_torque_limits", &DynamicsSolver::enable_torque_limits)
          .def("enable_self_collision_avoidance", &DynamicsSolver::enable_self_collision_avoidance)
          .def("configure_self_collision_avoidance", &DynamicsSolver::configure_self_collision_avoidance)
          .def("dump_status", &DynamicsSolver::dump_status)
          .def("set_static", &DynamicsSolver::set_static)
          .def("solve", &DynamicsSolver::solve)
          .def("remove_task", &DynamicsSolver::remove_task)
          .def("remove_contact", &DynamicsSolver::remove_contact)
          .add_property(
              "robot", +[](const DynamicsSolver& solver) { return solver.robot; })
          .def(
              "count_contacts", +[](DynamicsSolver& solver) { return solver.contacts.size(); })
          .def(
              "get_contact", +[](DynamicsSolver& solver, int i) -> Contact& { return *solver.contacts[i]; },
              return_internal_reference<>())
          .def<PositionTask& (DynamicsSolver::*)(std::string, Eigen::Vector3d)>(
              "add_position_task", &DynamicsSolver::add_position_task, return_internal_reference<>())
          .def<RelativePositionTask& (DynamicsSolver::*)(std::string, std::string, Eigen::Vector3d)>(
              "add_relative_position_task", &DynamicsSolver::add_relative_position_task, return_internal_reference<>())
          .def<OrientationTask& (DynamicsSolver::*)(std::string, Eigen::Matrix3d)>(
              "add_orientation_task", &DynamicsSolver::add_orientation_task, return_internal_reference<>())
          .def<JointsTask& (DynamicsSolver::*)()>("add_joints_task", &DynamicsSolver::add_joints_task,
                                                  return_internal_reference<>())
          .def<MimicTask& (DynamicsSolver::*)()>("add_mimic_task", &DynamicsSolver::add_mimic_task,
                                                 return_internal_reference<>())
          .def<CoMTask& (DynamicsSolver::*)(Eigen::Vector3d)>("add_com_task", &DynamicsSolver::add_com_task,
                                                              return_internal_reference<>())
          .def<FrameTask (DynamicsSolver::*)(std::string, Eigen::Affine3d)>("add_frame_task",
                                                                            &DynamicsSolver::add_frame_task);

  class_<Task, boost::noncopyable>("DynamicsTask", no_init)
      .add_property("name", &Task::name)
      .add_property("solver", &Task::solver)
      .add_property("weight", &Task::weight)
      .add_property(
          "priority", +[](Task& task) { return task.priority_name(); })
      .add_property("A", &Task::A)
      .add_property("b", &Task::b)
      .add_property("kp", &Task::kp, &Task::kp)
      .add_property("kd", &Task::kd, &Task::kd)
      .add_property("critically_damped", &Task::critically_damped, &Task::critically_damped)
      .add_property("error", &Task::error)
      .add_property("derror", &Task::derror)
      .def("update", &Task::update)
      .def("set_priority", &Task::set_priority)
      .def("set_weight", &Task::set_weight)
      .def("set_name", &Task::set_name)
      .def(
          "configure", +[](Task& task, std::string name, std::string priority, double weight) {
            task.configure(name, priority, weight);
          });

  class_<PositionTask, bases<Task>>("DynamicsPositionTask", init<RobotWrapper::FrameIndex, Eigen::Vector3d>())
      .add_property("frame_index", &PositionTask::frame_index)
      .add_property(
          "target_world", +[](const PositionTask& task) { return task.target_world; }, &PositionTask::target_world)
      .add_property(
          "dtarget_world", +[](const PositionTask& task) { return task.dtarget_world; }, &PositionTask::dtarget_world)
      .add_property("mask", &PositionTask::mask, &PositionTask::mask);

  class_<CoMTask, bases<Task>>("DynamicsCoMTask", init<Eigen::Vector3d>())
      .add_property(
          "target_world", +[](const CoMTask& task) { return task.target_world; }, &CoMTask::target_world)
      .add_property(
          "dtarget_world", +[](const CoMTask& task) { return task.dtarget_world; }, &CoMTask::dtarget_world)
      .add_property("mask", &CoMTask::mask, &CoMTask::mask);

  class_<RelativePositionTask, bases<Task>>("DynamicsRelativePositionTask",
                                            init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Vector3d>())
      .add_property(
          "target", +[](const RelativePositionTask& task) { return task.target; }, &RelativePositionTask::target)
      .add_property(
          "dtarget", +[](const RelativePositionTask& task) { return task.dtarget; }, &RelativePositionTask::dtarget)
      .add_property("mask", &RelativePositionTask::mask, &RelativePositionTask::mask);

  class_<OrientationTask, bases<Task>>("DynamicsOrientationTask", init<RobotWrapper::FrameIndex, Eigen::Matrix3d>())
      .add_property(
          "R_world_frame", +[](const OrientationTask& task) { return task.R_world_frame; },
          &OrientationTask::R_world_frame)
      .add_property(
          "omega_world", +[](const OrientationTask& task) { return task.omega_world; }, &OrientationTask::omega_world)
      .add_property("mask", &OrientationTask::mask, &OrientationTask::mask);

  class_<FrameTask>("DynamicsFrameTask", init<>())
      .def(
          "position", +[](const FrameTask& task) -> PositionTask& { return *task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const FrameTask& task) -> OrientationTask& { return *task.orientation; },
          return_internal_reference<>())
      .def("configure", &FrameTask::configure)
      .add_property("T_world_frame", &FrameTask::get_T_world_frame, &FrameTask::set_T_world_frame);

  class_<JointsTask, bases<Task>>("DynamicsJointsTask", init<>())
      .def("set_joint", &JointsTask::set_joint)
      .def(
          "set_joints", +[](JointsTask& task,
                            boost::python::dict& py_dict) { update_map<std::string, double>(task.joints, py_dict); })
      .def(
          "set_joints_velocities", +[](JointsTask& task, boost::python::dict& py_dict) {
            update_map<std::string, double>(task.djoints, py_dict);
          });

  class_<MimicTask, bases<Task>>("DynamicsMimicTask", init<>()).def("set_mimic", &MimicTask::set_mimic);
}