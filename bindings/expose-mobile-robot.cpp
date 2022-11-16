#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/model/mobile_robot.h"
#include "placo/model/legged_robot.h"
#include "placo/control/kinematics_solver.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

template <typename RobotType>
class_<RobotType>& exposeRobotType(const char* name)
{
  return class_<RobotType>(name, init<std::string>())
      .add_property("state", &RobotType::state)
      .add_property("model", &RobotType::model)
      .add_property("collision_model", &RobotType::collision_model)
      .add_property("visual_model", &RobotType::visual_model)
      .def("load_collisions_pairs", &RobotType::load_collisions_pairs)
      .def("reset", &RobotType::reset)
      .def("neutral_state", &RobotType::neutral_state)
      .def("set_joint", &RobotType::set_joint)
      .def("get_joint", &RobotType::get_joint)
      .def("update_kinematics", &RobotType::update_kinematics)
      .def("get_T_world_fbase", &RobotType::get_T_world_fbase)
      .def("set_T_world_fbase", &RobotType::set_T_world_fbase)
      .def("com_world", &RobotType::com_world)
      .def("joint_names", &RobotType::joint_names)
      .def("frame_names", &RobotType::frame_names)
      .def("self_collisions", &RobotType::self_collisions)
      .def("com_jacobian", &RobotType::com_jacobian)
      .def(
          "get_T_world_frame",
          +[](RobotType& robot, const std::string& frame) { return robot.get_T_world_frame(frame); })
      .def(
          "set_T_world_frame", +[](RobotType& robot, const std::string& frame,
                                   Eigen::Affine3d T_world_frame) { robot.set_T_world_frame(frame, T_world_frame); })
      .def(
          "frame_jacobian", +[](RobotType& robot, const std::string& frame) { return robot.frame_jacobian(frame); })
      .def(
          "make_solver", +[](RobotType& robot) { return KinematicsSolver(&robot); });
}

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

  exposeRobotType<MobileRobot>("MobileRobot");
  exposeRobotType<LeggedRobot>("LeggedRobot");

  exposeStdVector<MobileRobot::Collision>("vector_Collision");
}