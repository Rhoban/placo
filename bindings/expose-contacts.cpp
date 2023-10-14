#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/model/robot_wrapper.h"
#include "placo/dynamics/dynamics_solver.h"
#include "placo/dynamics/position_task.h"
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

  class_<DynamicsSolver::Contact>("DynamicsSolverContact")
      .def(
          "configure",
          +[](DynamicsSolver::Contact& contact, const std::string& frame_name, std::string type, double mu = 1.0,
              double length = 1.0, double width = 1.0) {
            DynamicsSolver::Contact::Type contact_type;

            if (type == "fixed")
            {
              contact_type = DynamicsSolver::Contact::Fixed;
            }
            else if (type == "planar")
            {
              contact_type = DynamicsSolver::Contact::Planar;
            }
            else if (type == "point")
            {
              contact_type = DynamicsSolver::Contact::Point;
            }
            else
            {
              throw std::runtime_error("Unknown contact type");
            }
            contact.configure(frame_name, contact_type, mu, length, width);
          },
          (boost::python::arg("frame_name"), boost::python::arg("type"), boost::python::arg("mu") = 1.0,
           boost::python::arg("length") = 1.0, boost::python::arg("width") = 1.0))
      .def_readwrite("frame_name", &DynamicsSolver::Contact::frame_name)
      .def_readwrite("type", &DynamicsSolver::Contact::type)
      .def_readwrite("length", &DynamicsSolver::Contact::length)
      .def_readwrite("width", &DynamicsSolver::Contact::width)
      .def_readwrite("mu", &DynamicsSolver::Contact::mu)
      .def_readwrite("weight_forces", &DynamicsSolver::Contact::weight_forces)
      .def_readwrite("weight_moments", &DynamicsSolver::Contact::weight_moments)
      .def("zmp", &DynamicsSolver::Contact::zmp)
      .add_property(
          "wrench", +[](DynamicsSolver::Contact& contact) { return contact.wrench; });

  class_<DynamicsSolver> solver_class =
      class_<DynamicsSolver>("DynamicsSolver", init<RobotWrapper&>())
          .def("add_contact", &DynamicsSolver::add_contact, return_internal_reference<>())
          .def("set_passive", &DynamicsSolver::set_passive)
          .def("add_loop_closing_constraint", &DynamicsSolver::add_loop_closing_constraint)
          .def("solve", &DynamicsSolver::solve)
          .def<PositionTask& (DynamicsSolver::*)(std::string, Eigen::Vector3d)>(
              "add_position_task", &DynamicsSolver::add_position_task, return_internal_reference<>())
          .def<OrientationTask& (DynamicsSolver::*)(std::string, Eigen::Matrix3d)>(
              "add_orientation_task", &DynamicsSolver::add_orientation_task, return_internal_reference<>())
          .add_property(
              "qdd_desired", +[](DynamicsSolver& id) { return id.qdd_desired; },
              +[](DynamicsSolver& id, const Eigen::VectorXd& qdd_desired) { id.qdd_desired = qdd_desired; });

  solver_class_ptr = &solver_class;

  registerTaskMethods(
      class_<PositionTask>("DynamicsPositionTask", init<RobotWrapper::FrameIndex, Eigen::Vector3d>())
          .add_property(
              "target_world", +[](const PositionTask& task) { return task.target_world; }, &PositionTask::target_world)
          .add_property("mask", &PositionTask::mask, &PositionTask::mask));

  registerTaskMethods(
      class_<OrientationTask>("DynamicsOrientationTask", init<RobotWrapper::FrameIndex, Eigen::Matrix3d>())
          .add_property(
              "R_world_frame", +[](const OrientationTask& task) { return task.R_world_frame; },
              &OrientationTask::R_world_frame)
          .add_property("mask", &OrientationTask::mask, &OrientationTask::mask));
}