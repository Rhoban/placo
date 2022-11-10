#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/model/mobile_robot.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeMobileRobot()
{
  class_<MobileRobot::State>("MobileRobot_State")
      .add_property(
          "q", +[](const MobileRobot::State& state) { return state.q; },
          +[](MobileRobot::State& state, const Eigen::VectorXd& q) { state.q = q; })
      .add_property(
          "qd", +[](const MobileRobot::State& state) { return state.qd; },
          +[](MobileRobot::State& state, const Eigen::VectorXd& qd) { state.qd = qd; });

  class_<MobileRobot::Collision>("Collision")
      .add_property("bodyA", &MobileRobot::Collision::bodyA)
      .add_property("bodyB", &MobileRobot::Collision::bodyB)
      .add_property("contacts", &MobileRobot::Collision::contacts);

  class_<MobileRobot>("MobileRobot", init<std::string>())
      .add_property("state", &MobileRobot::state)
      .add_property("model", &MobileRobot::model)
      .add_property("collision_model", &MobileRobot::collision_model)
      .add_property("visual_model", &MobileRobot::visual_model)
      .def("load_collisions_pairs", &MobileRobot::load_collisions_pairs)
      .def("reset", &MobileRobot::reset)
      .def("neutral_state", &MobileRobot::neutral_state)
      .def("set_joint", &MobileRobot::set_joint)
      .def("update_kinematics", &MobileRobot::update_kinematics)
      .def("get_T_world_fbase", &MobileRobot::get_T_world_fbase)
      .def("set_T_world_fbase", &MobileRobot::set_T_world_fbase)
      .def("com_world", &MobileRobot::com_world)
      .def("joint_names", &MobileRobot::joint_names)
      .def("frame_names", &MobileRobot::frame_names)
      .def("self_collisions", &MobileRobot::self_collisions)
      .def<Eigen::Affine3d (MobileRobot::*)(const std::string&)>("get_T_world_frame", &MobileRobot::get_T_world_frame)
      .def<void (MobileRobot::*)(const std::string&, Eigen::Affine3d)>("set_T_world_frame",
                                                                       &MobileRobot::set_T_world_frame)
      .def<Eigen::MatrixXd (MobileRobot::*)(const std::string&)>("frame_jacobian", &MobileRobot::frame_jacobian)
      .def("com_jacobian", &MobileRobot::com_jacobian);

  exposeStdVector<MobileRobot::Collision>("vector_Collision");
}