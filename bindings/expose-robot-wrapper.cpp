#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/model/robot_wrapper.h"
#include "placo/model/humanoid_robot.h"
#include "placo/control/kinematics_solver.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

template <typename RobotType>
class_<RobotType> exposeRobotType(const char* name)
{
  return class_<RobotType>(name, init<std::string>())
      .add_property("state", &RobotType::state)
      .add_property("model", &RobotType::model)
      .add_property("collision_model", &RobotType::collision_model)
      .add_property("visual_model", &RobotType::visual_model)
      .def("load_collisions_pairs", &RobotType::load_collisions_pairs)
      .def("get_joint_offset", &RobotType::get_joint_offset)
      .def("get_joint_v_offset", &RobotType::get_joint_v_offset)
      .def("reset", &RobotType::reset)
      .def("neutral_state", &RobotType::neutral_state)
      .def("set_joint", &RobotType::set_joint)
      .def("get_joint", &RobotType::get_joint)
      .def("update_kinematics", &RobotType::update_kinematics)
      .def("get_T_world_fbase", &RobotType::get_T_world_fbase)
      .def("set_T_world_fbase", &RobotType::set_T_world_fbase)
      .def("com_world", &RobotType::com_world)
      .def("joint_names", &RobotType::joint_names)
      .def("actuated_joint_names", &RobotType::actuated_joint_names)
      .def("frame_names", &RobotType::frame_names)
      .def("self_collisions", &RobotType::self_collisions)
      .def("distances", &RobotType::distances)
      .def("com_jacobian", &RobotType::com_jacobian)
      .def("generalized_gravity", &RobotType::generalized_gravity)
      .def("non_linear_effects", &RobotType::non_linear_effects)
      .def("mass_matrix", &RobotType::mass_matrix)
      .def("integrate", &RobotType::integrate)
      .def(
          "static_gravity_compensation_torques",
          +[](RobotType& robot, const std::string& frame) { return robot.static_gravity_compensation_torques(frame); })
      .def(
          "static_gravity_compensation_torques_dict",
          +[](RobotType& robot, const std::string& frame) {
            auto torques = robot.static_gravity_compensation_torques(frame);
            boost::python::dict dict;

            for (auto& dof : robot.actuated_joint_names())
            {
              dict[dof] = torques[robot.get_joint_v_offset(dof)];
            }

            return dict;
          })
      .def(
          "get_T_world_frame",
          +[](RobotType& robot, const std::string& frame) { return robot.get_T_world_frame(frame); })
      .def(
          "set_T_world_frame", +[](RobotType& robot, const std::string& frame,
                                   Eigen::Affine3d T_world_frame) { robot.set_T_world_frame(frame, T_world_frame); })
      .def(
          "frame_jacobian", +[](RobotType& robot, const std::string& frame,
                                const std::string& reference) { return robot.frame_jacobian(frame, reference); })
      .def(
          "joint_jacobian", +[](RobotType& robot, const std::string& joint,
                                const std::string& reference) { return robot.joint_jacobian(joint, reference); })
      .def(
          "make_solver", +[](RobotType& robot) { return KinematicsSolver(&robot); });
}

void exposeRobotWrapper()
{
  class_<RobotWrapper::State>("RobotWrapper_State")
      .add_property(
          "q", +[](const RobotWrapper::State& state) { return state.q; },
          +[](RobotWrapper::State& state, const Eigen::VectorXd& q) { state.q = q; })
      .add_property(
          "qd", +[](const RobotWrapper::State& state) { return state.qd; },
          +[](RobotWrapper::State& state, const Eigen::VectorXd& qd) { state.qd = qd; });

  class_<RobotWrapper::Collision>("Collision")
      .add_property("bodyA", &RobotWrapper::Collision::bodyA)
      .add_property("bodyB", &RobotWrapper::Collision::bodyB)
      .add_property("parentA", &RobotWrapper::Collision::parentA)
      .add_property("parentB", &RobotWrapper::Collision::parentB)
      .def(
          "get_contact", +[](RobotWrapper::Collision& collision, int index) { return collision.contacts[index]; });

  class_<RobotWrapper::Distance>("Distance")
      .add_property("parentA", &RobotWrapper::Distance::parentA)
      .add_property("parentB", &RobotWrapper::Distance::parentB)
      .add_property(
          "pointA", +[](RobotWrapper::Distance& distance) { return distance.pointA; })
      .add_property(
          "pointB", +[](RobotWrapper::Distance& distance) { return distance.pointB; })
      .add_property(
          "normal", +[](RobotWrapper::Distance& distance) { return distance.normal; })
      .add_property("min_distance", &RobotWrapper::Distance::min_distance);

  exposeRobotType<RobotWrapper>("RobotWrapper");

  exposeRobotType<HumanoidRobot>("HumanoidRobot")
      .def<void (HumanoidRobot::*)(const std::string&)>("update_support_side", &HumanoidRobot::update_support_side)
      .def("ensure_on_floor", &HumanoidRobot::ensure_on_floor)
      .def("get_T_world_left", &HumanoidRobot::get_T_world_left)
      .def("get_T_world_right", &HumanoidRobot::get_T_world_right)
      .def("get_T_world_trunk", &HumanoidRobot::get_T_world_trunk)
      .def("get_com_velocity", &HumanoidRobot::get_com_velocity)
      .def("dcm", &HumanoidRobot::dcm)
      .def("zmp", &HumanoidRobot::zmp)
      .def("other_side", &HumanoidRobot::other_side)
      .def("update_trunk_angular_velocity", &HumanoidRobot::update_trunk_angular_velocity)
      .def(
          "get_omega", +[](const HumanoidRobot& robot) { return robot.omega_b; })
      .def(
          "get_support_side", +[](const HumanoidRobot& robot) { return robot.support_side; })
      .def(
          "get_flying_side", +[](const HumanoidRobot& robot) { return robot.flying_side; });

  exposeStdVector<RobotWrapper::Collision>("vector_Collision");
  exposeStdVector<RobotWrapper::Distance>("vector_Distance");
}