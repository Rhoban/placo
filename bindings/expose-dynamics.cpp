#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"

#include "doxystub.h"
#include "placo/model/robot_wrapper.h"
#include "placo/dynamics/dynamics_solver.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigen-to-python.hpp>

using namespace placo;
using namespace placo::dynamics;
using namespace placo::model;

// Overloads
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(frame_configure_overloads, configure, 2, 4);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(solve_overloads, solve, 0, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(set_torque_overloads, set_torque, 2, 4);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(set_joint_overloads, set_joint, 2, 4);

void exposeDynamics()
{
  class__<DynamicsSolver::Result>("DynamicsSolverResult")
      .add_property("success", &DynamicsSolver::Result::success)
      .def_readwrite("tau", &DynamicsSolver::Result::tau)
      .def_readwrite("qdd", &DynamicsSolver::Result::qdd)
      .def_readwrite("tau_contacts", &DynamicsSolver::Result::tau_contacts)
      .def(
          "tau_dict", +[](const DynamicsSolver::Result& result, RobotWrapper& robot) {
            boost::python::dict dict;

            for (auto& dof : robot.joint_names())
            {
              dict[dof] = result.tau[robot.get_joint_v_offset(dof)];
            }

            return dict;
          });

  class__<Contact, boost::noncopyable>("Contact", no_init)
      .def_readwrite("active", &Contact::active)
      .def_readwrite("mu", &Contact::mu)
      .def_readwrite("weight_forces", &Contact::weight_forces)
      .def_readwrite("weight_tangentials", &Contact::weight_tangentials)
      .def_readwrite("weight_moments", &Contact::weight_moments)
      .def_readonly("wrench", &Contact::wrench);

  class__<PointContact, bases<Contact>>("PointContact", init<PositionTask&, bool>())
      .def(
          "position_task", +[](PointContact& contact) -> PositionTask& { return *contact.position_task; },
          return_internal_reference<>())
      .def_readwrite("unilateral", &PointContact::unilateral)
      .def_readwrite("R_world_surface", &PointContact::R_world_surface);

  class__<Contact6D, bases<Contact>>("Contact6D", init<FrameTask&, bool>())
      .def(
          "position_task", +[](Contact6D& contact) -> PositionTask& { return *contact.position_task; },
          return_internal_reference<>())
      .def(
          "orientation_task", +[](Contact6D& contact) -> OrientationTask& { return *contact.orientation_task; },
          return_internal_reference<>())
      .def_readwrite("unilateral", &Contact6D::unilateral)
      .def_readwrite("length", &Contact6D::length)
      .def_readwrite("width", &Contact6D::width)
      .def("zmp", &Contact6D::zmp);

  class__<LineContact, bases<Contact>>("LineContact", init<FrameTask&, bool>())
      .def(
          "position_task", +[](LineContact& contact) -> PositionTask& { return *contact.position_task; },
          return_internal_reference<>())
      .def(
          "orientation_task", +[](LineContact& contact) -> OrientationTask& { return *contact.orientation_task; },
          return_internal_reference<>())
      .def_readwrite("unilateral", &LineContact::unilateral)
      .def_readwrite("length", &LineContact::length)
      .def("zmp", &LineContact::zmp)
      .add_property(
          "R_world_surface", +[](PointContact& contact) { return contact.R_world_surface; },
          +[](PointContact& contact, Eigen::Matrix3d R) { contact.R_world_surface = R; });

  class__<ExternalWrenchContact, bases<Contact>>("ExternalWrenchContact", init<RobotWrapper::FrameIndex>())
      .add_property("frame_index", &ExternalWrenchContact::frame_index)
      .def_readwrite("w_ext", &ExternalWrenchContact::w_ext);

  class__<PuppetContact, bases<Contact>>("PuppetContact", init<>());

  class__<TaskContact, bases<Contact>>("TaskContact", init<Task&>());

  class__<DynamicsSolver>("DynamicsSolver", init<RobotWrapper&>())
      .add_property("problem", &DynamicsSolver::problem)
      .def_readwrite("damping", &DynamicsSolver::damping)
      .def_readwrite("dt", &DynamicsSolver::dt)
      .def("set_qdd_safe", &DynamicsSolver::set_qdd_safe)
      .def("set_torque_limit", &DynamicsSolver::set_torque_limit)
      .def_readwrite("gravity_only", &DynamicsSolver::gravity_only)
      .def_readwrite("torque_cost", &DynamicsSolver::torque_cost)
      .add_property(
          "extra_force", +[](DynamicsSolver& solver) { return solver.extra_force; },
          +[](DynamicsSolver& solver, Eigen::VectorXd force) { solver.extra_force = force; })
      .def("mask_fbase", &DynamicsSolver::mask_fbase)
      .def("add_point_contact", &DynamicsSolver::add_point_contact, return_internal_reference<>())
      .def("add_unilateral_point_contact", &DynamicsSolver::add_unilateral_point_contact, return_internal_reference<>())
      .def("add_planar_contact", &DynamicsSolver::add_planar_contact, return_internal_reference<>())
      .def("add_fixed_contact", &DynamicsSolver::add_fixed_contact, return_internal_reference<>())
      .def("add_line_contact", &DynamicsSolver::add_line_contact, return_internal_reference<>())
      .def("add_unilateral_line_contact", &DynamicsSolver::add_unilateral_line_contact, return_internal_reference<>())
      .def<ExternalWrenchContact& (DynamicsSolver::*)(std::string, std::string)>(
          "add_external_wrench_contact", &DynamicsSolver::add_external_wrench_contact, return_internal_reference<>())
      .def("add_puppet_contact", &DynamicsSolver::add_puppet_contact, return_internal_reference<>())
      .def("add_task_contact", &DynamicsSolver::add_task_contact, return_internal_reference<>())
      .def("add_avoid_self_collisions_constraint", &DynamicsSolver::add_avoid_self_collisions_constraint,
           return_internal_reference<>())
      .def("enable_velocity_limits", &DynamicsSolver::enable_velocity_limits)
      .def("enable_velocity_vs_torque_limits", &DynamicsSolver::enable_velocity_vs_torque_limits)
      .def("enable_joint_limits", &DynamicsSolver::enable_joint_limits)
      .def("enable_torque_limits", &DynamicsSolver::enable_torque_limits)
      .def("dump_status", &DynamicsSolver::dump_status)
      .def("solve", &DynamicsSolver::solve, solve_overloads())
      .def<void (DynamicsSolver::*)(Task&)>("add_task", &DynamicsSolver::add_task)
      .def<void (DynamicsSolver::*)(Constraint&)>("add_constraint", &DynamicsSolver::add_constraint)
      .def<void (DynamicsSolver::*)(Constraint&)>("add_constraint", &DynamicsSolver::add_constraint)
      .def("clear", &DynamicsSolver::clear)
      .def<void (DynamicsSolver::*)(Task&)>("remove_task", &DynamicsSolver::remove_task)
      .def<void (DynamicsSolver::*)(FrameTask&)>("remove_task", &DynamicsSolver::remove_task)
      .def("remove_contact", &DynamicsSolver::remove_contact)
      .def("remove_constraint", &DynamicsSolver::remove_constraint)
      .def("set_kp", &DynamicsSolver::set_kp)
      .def("set_kd", &DynamicsSolver::set_kd)
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
      .def<RelativeOrientationTask& (DynamicsSolver::*)(std::string, std::string, Eigen::Matrix3d)>(
          "add_relative_orientation_task", &DynamicsSolver::add_relative_orientation_task,
          return_internal_reference<>())
      .def<RelativeFrameTask (DynamicsSolver::*)(std::string, std::string, Eigen::Affine3d)>(
          "add_relative_frame_task", &DynamicsSolver::add_relative_frame_task)
      .def<JointsTask& (DynamicsSolver::*)()>("add_joints_task", &DynamicsSolver::add_joints_task,
                                              return_internal_reference<>())
      .def<TorqueTask& (DynamicsSolver::*)()>("add_torque_task", &DynamicsSolver::add_torque_task,
                                              return_internal_reference<>())
      .def<GearTask& (DynamicsSolver::*)()>("add_gear_task", &DynamicsSolver::add_gear_task,
                                            return_internal_reference<>())
      .def<CoMTask& (DynamicsSolver::*)(Eigen::Vector3d)>("add_com_task", &DynamicsSolver::add_com_task,
                                                          return_internal_reference<>())

      .def<OrientationTask& (DynamicsSolver::*)(std::string, Eigen::Matrix3d)>(
          "add_orientation_task", &DynamicsSolver::add_orientation_task, return_internal_reference<>())
      .def<FrameTask (DynamicsSolver::*)(std::string, Eigen::Affine3d)>("add_frame_task",
                                                                        &DynamicsSolver::add_frame_task);

  class__<Task, bases<tools::Prioritized>, boost::noncopyable>("DynamicsTask", no_init)
      .def_readwrite("A", &Task::A)
      .def_readwrite("b", &Task::b)
      .add_property("kp", &Task::kp, &Task::kp)
      .add_property("kd", &Task::kd, &Task::kd)
      .add_property("error", &Task::error)
      .add_property("derror", &Task::derror);

  class__<PositionTask, bases<Task>>("DynamicsPositionTask", init<RobotWrapper::FrameIndex, Eigen::Vector3d>())
      .add_property("frame_index", &PositionTask::frame_index)
      .def_readwrite("target_world", &PositionTask::target_world)
      .def_readwrite("dtarget_world", &PositionTask::dtarget_world)
      .def_readwrite("ddtarget_world", &PositionTask::ddtarget_world)
      .add_property("mask", &PositionTask::mask, &PositionTask::mask);

  class__<CoMTask, bases<Task>>("DynamicsCoMTask", init<Eigen::Vector3d>())
      .def_readwrite("target_world", &CoMTask::target_world)
      .def_readwrite("dtarget_world", &CoMTask::dtarget_world)
      .def_readwrite("ddtarget_world", &CoMTask::ddtarget_world)
      .add_property("mask", &CoMTask::mask, &CoMTask::mask);

  class__<RelativePositionTask, bases<Task>>(
      "DynamicsRelativePositionTask", init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Vector3d>())
      .def_readwrite("target", &RelativePositionTask::target)
      .def_readwrite("dtarget", &RelativePositionTask::dtarget)
      .def_readwrite("ddtarget", &RelativePositionTask::ddtarget)
      .add_property("mask", &RelativePositionTask::mask, &RelativePositionTask::mask);

  class__<RelativeOrientationTask, bases<Task>>(
      "DynamicsRelativeOrientationTask", init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Matrix3d>())
      .def_readwrite("R_a_b", &RelativeOrientationTask::R_a_b)
      .def_readwrite("omega_a_b", &RelativeOrientationTask::omega_a_b)
      .def_readwrite("domega_a_b", &RelativeOrientationTask::domega_a_b)
      .add_property("mask", &RelativeOrientationTask::mask, &RelativeOrientationTask::mask);

  class__<OrientationTask, bases<Task>>("DynamicsOrientationTask", init<RobotWrapper::FrameIndex, Eigen::Matrix3d>())
      .def_readwrite("R_world_frame", &OrientationTask::R_world_frame)
      .def_readwrite("omega_world", &OrientationTask::omega_world)
      .def_readwrite("domega_world", &OrientationTask::domega_world)
      .add_property("mask", &OrientationTask::mask, &OrientationTask::mask);

  class__<FrameTask>("DynamicsFrameTask", init<>())
      .def(
          "position", +[](const FrameTask& task) -> PositionTask& { return *task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const FrameTask& task) -> OrientationTask& { return *task.orientation; },
          return_internal_reference<>())
      .def("configure", &FrameTask::configure, frame_configure_overloads())
      .add_property("T_world_frame", &FrameTask::get_T_world_frame, &FrameTask::set_T_world_frame);

  class__<RelativeFrameTask>("DynamicsRelativeFrameTask", init<>())
      .def(
          "position", +[](const RelativeFrameTask& task) -> RelativePositionTask& { return *task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const RelativeFrameTask& task) -> RelativeOrientationTask& { return *task.orientation; },
          return_internal_reference<>())
      .def("configure", &RelativeFrameTask::configure)
      .add_property("T_a_b", &RelativeFrameTask::get_T_a_b, &RelativeFrameTask::set_T_a_b);

  class__<JointsTask, bases<Task>>("DynamicsJointsTask", init<>())
      .def("set_joint", &JointsTask::set_joint, set_joint_overloads())
      .def("get_joint", &JointsTask::get_joint)
      .def(
          "set_joints", +[](JointsTask& task,
                            boost::python::dict& py_dict) { update_map<std::string, double>(task.joints, py_dict); })
      .def(
          "set_joints_velocities", +[](JointsTask& task, boost::python::dict& py_dict) {
            update_map<std::string, double>(task.djoints, py_dict);
          });

  class__<TorqueTask, bases<Task>>("DynamicsTorqueTask", init<>())
      .def("set_torque", &TorqueTask::set_torque, set_torque_overloads())
      .def("reset_torque", &TorqueTask::reset_torque);

  class__<GearTask, bases<Task>>("DynamicsGearTask", init<>())
      .def("set_gear", &GearTask::set_gear)
      .def("add_gear", &GearTask::add_gear);

  class__<Constraint, bases<tools::Prioritized>, boost::noncopyable>("DynamicsConstraint", no_init);

  class__<AvoidSelfCollisionsConstraint, bases<Constraint>>("AvoidSelfCollisionsDynamicsConstraint", init<>())
      .def_readwrite("self_collisions_margin", &AvoidSelfCollisionsConstraint::self_collisions_margin)
      .def_readwrite("self_collisions_trigger", &AvoidSelfCollisionsConstraint::self_collisions_trigger);
}