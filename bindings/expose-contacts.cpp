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
          "tau_dict",
          +[](const GravityTorques::Result& result, RobotWrapper& robot) {
            boost::python::dict dict;

            for (auto& dof : robot.actuated_joint_names())
            {
              dict[dof] = result.tau[robot.get_joint_v_offset(dof)];
            }

            return dict;
          })
      .add_property(
          "contact_wrenches", +[](const GravityTorques::Result& result) { return result.contact_wrenches; });

  class_<GravityTorques>("GravityTorques")
      .def("compute_gravity_torques", &GravityTorques::compute_gravity_torques)
      .staticmethod("compute_gravity_torques");
}