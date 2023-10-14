#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/model/robot_wrapper.h"
#include "placo/dynamics/inverse_dynamics.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace placo;

void exposeContacts()
{
  class_<InverseDynamics::Result>("InverseDynamicsResult")
      .add_property("success", &InverseDynamics::Result::success)
      .add_property(
          "tau", +[](const InverseDynamics::Result& result) { return result.tau; })
      .add_property(
          "qdd", +[](const InverseDynamics::Result& result) { return result.qdd; })
      .def(
          "tau_dict", +[](const InverseDynamics::Result& result, RobotWrapper& robot) {
            boost::python::dict dict;

            for (auto& dof : robot.actuated_joint_names())
            {
              dict[dof] = result.tau[robot.get_joint_v_offset(dof)];
            }

            return dict;
          });

  class_<InverseDynamics::Contact>("InverseDynamicsContact")
      .def(
          "configure",
          +[](InverseDynamics::Contact& contact, const std::string& frame_name, std::string type, double mu = 1.0,
              double length = 1.0, double width = 1.0) {
            InverseDynamics::Contact::Type contact_type;

            if (type == "fixed")
            {
              contact_type = InverseDynamics::Contact::Fixed;
            }
            else if (type == "planar")
            {
              contact_type = InverseDynamics::Contact::Planar;
            }
            else if (type == "point")
            {
              contact_type = InverseDynamics::Contact::Point;
            }
            else
            {
              throw std::runtime_error("Unknown contact type");
            }
            contact.configure(frame_name, contact_type, mu, length, width);
          },
          (boost::python::arg("frame_name"), boost::python::arg("type"), boost::python::arg("mu") = 1.0,
           boost::python::arg("length") = 1.0, boost::python::arg("width") = 1.0))
      .def_readwrite("frame_name", &InverseDynamics::Contact::frame_name)
      .def_readwrite("type", &InverseDynamics::Contact::type)
      .def_readwrite("length", &InverseDynamics::Contact::length)
      .def_readwrite("width", &InverseDynamics::Contact::width)
      .def_readwrite("mu", &InverseDynamics::Contact::mu)
      .def_readwrite("weight_forces", &InverseDynamics::Contact::weight_forces)
      .def_readwrite("weight_moments", &InverseDynamics::Contact::weight_moments)
      .def("zmp", &InverseDynamics::Contact::zmp)
      .add_property(
          "wrench", +[](InverseDynamics::Contact& contact) { return contact.wrench; });

  class_<InverseDynamics>("InverseDynamics", init<RobotWrapper&>())
      .def("add_contact", &InverseDynamics::add_contact, return_internal_reference<>())
      .def("set_passive", &InverseDynamics::set_passive)
      .def("add_loop_closing_constraint", &InverseDynamics::add_loop_closing_constraint)
      .def("compute", &InverseDynamics::compute)
      .add_property(
          "qdd_desired", +[](InverseDynamics& id) { return id.qdd_desired; },
          +[](InverseDynamics& id, const Eigen::VectorXd& qdd_desired) { id.qdd_desired = qdd_desired; });
}