#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/model/robot_wrapper.h"
#include "placo/contacts/gravity_torques.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace placo;

void exposeContacts()
{
  class_<GravityTorques::Result>("GravityTorquesResult")
      .add_property("success", &GravityTorques::Result::success)
      .add_property(
          "tau", +[](const GravityTorques::Result& result) { return result.tau; })

      .def(
          "tau_dict", +[](const GravityTorques::Result& result, RobotWrapper& robot) {
            boost::python::dict dict;

            for (auto& dof : robot.actuated_joint_names())
            {
              dict[dof] = result.tau[robot.get_joint_v_offset(dof)];
            }

            return dict;
          });

  class_<GravityTorques::Contact>("GravityTorquesContact")
      .def(
          "configure",
          +[](GravityTorques::Contact& contact, const std::string& frame_name, std::string type, double mu = 1.0,
              double length = 1.0, double width = 1.0) {
            if (type == "planar")
            {
              contact.configure(frame_name, GravityTorques::Contact::Planar, mu, length, width);
            }
            else if (type == "point")
            {
              contact.configure(frame_name, GravityTorques::Contact::Point, mu, length, width);
            }
            else
            {
              throw std::runtime_error("Unknown contact type");
            }
          },
          (boost::python::arg("frame_name"), boost::python::arg("type"), boost::python::arg("mu") = 1.0,
           boost::python::arg("length") = 1.0, boost::python::arg("width") = 1.0))
      .def_readwrite("frame_name", &GravityTorques::Contact::frame_name)
      .def_readwrite("type", &GravityTorques::Contact::type)
      .def_readwrite("length", &GravityTorques::Contact::length)
      .def_readwrite("width", &GravityTorques::Contact::width)
      .def_readwrite("mu", &GravityTorques::Contact::mu)
      .def_readwrite("weight_forces", &GravityTorques::Contact::weight_forces)
      .def_readwrite("weight_moments", &GravityTorques::Contact::weight_moments)
      .add_property(
          "wrench", +[](GravityTorques::Contact& contact) { return contact.wrench; });

  class_<GravityTorques>("GravityTorques", init<RobotWrapper&>())
      .def("add_contact", &GravityTorques::add_contact, return_internal_reference<>())
      .def("set_passive", &GravityTorques::set_passive)
      .def("add_loop_closing_constraint", &GravityTorques::add_loop_closing_constraint)
      .def("compute", &GravityTorques::compute);
}