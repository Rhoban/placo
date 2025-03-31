#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "doxystub/registry.h"
#include "placo/model/robot_wrapper.h"
#include "placo/humanoid/humanoid_robot.h"
#include "placo/kinematics/kinematics_solver.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigen-to-python.hpp>

using namespace boost::python;
using namespace placo;
using namespace placo::model;
using namespace placo::humanoid;

#ifdef HAVE_RHOBAN_UTILS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(read_from_histories_overloads, read_from_histories, 2, 5);
#endif

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(joint_names_overloads, joint_names, 0, 1);

template <class RobotType, class W1>
void exposeRobotType(class_<RobotType, W1>& type)
{
  type.add_property("state", &RobotType::state, typed_docstring<RobotWrapper::State>("Robot state"))
      .add_property("model", &RobotType::model, typed_docstring<pinocchio::Model>("Pinocchio model"))
      .add_property("collision_model", &RobotType::collision_model,
                    typed_docstring<pinocchio::GeometryModel>("Pinocchio geometry model"))
      .add_property("visual_model", &RobotType::visual_model,
                    typed_docstring<pinocchio::GeometryModel>("Pinocchio visual model"))
      .def("load_collision_pairs", &RobotType::load_collision_pairs, args("self", "filename"),
           "Load collision pairs from given `filename` (JSON)")
      .def("get_joint_offset", &RobotType::get_joint_offset, args("self", "joint_name"),
           "Returns the offset of the joint in `state.q` for given `joint_name`")
      .def("get_joint_v_offset", &RobotType::get_joint_v_offset, args("self", "joint_name"),
           "Returns the offset of the joint in `state.qd` for given `joint_name`")
      .def("reset", &RobotType::reset, args("self"), "Resets the robot's internal state")
      .def("neutral_state", &RobotType::neutral_state, args("self"), "Robot neutral initial styate")
      .def("set_joint", &RobotType::set_joint, args("self", "joint_name", "q"),
           "Sets the joint value `q` for `joint_name`")
      .def("get_joint", &RobotType::get_joint, args("self", "joint_name"), "Gets the joint value for `joint_name`")
      .def("set_joint_velocity", &RobotType::set_joint_velocity, args("self", "joint_name", "qd"),
           "Sets the joint velocity `q` for `joint_name`")
      .def("get_joint_velocity", &RobotType::get_joint_velocity, args("self", "joint_name"),
           "Gets the joint velocity for `joint_name`")
      .def("set_joint_acceleration", &RobotType::set_joint_acceleration, args("self", "joint_name", "qdd"),
           "Sets the joint acceleration to `qdd` for `joint_name`")
      .def("get_joint_acceleration", &RobotType::get_joint_acceleration, args("self", "joint_name"),
           "Gets the joint acceleration for `joint_name`")
      .def("set_velocity_limit", &RobotType::set_velocity_limit, args("self", "joint_name", "limit"),
           "Sets the joint velocity limit to `limit` for `joint_name`")
      .def("set_velocity_limits", &RobotType::set_velocity_limits, args("self", "limit"),
           "Sets the joint velocity limit to `limit` for all joints")
      .def("set_torque_limit", &RobotType::set_torque_limit, args("self", "joint_name", "limit"), "Sets the torque limit to `limit` for `joint_name`")
      .def("set_joint_limits", &RobotType::set_joint_limits, args("self", "joint_name", "lower", "upper"), "Sets the joint limit to [`lower`, `upper`] for joint `joint_name`"))
      .def("update_kinematics", &RobotType::update_kinematics, args("self"), "Recomputes all kinematics quantities")
      .def("compute_hessians", &RobotType::compute_hessians, args("self"), "Computes hessians")
      .def(
          "get_frame_hessian",
          +[](RobotType& robot, const std::string frame, const std::string joint) {
    return robot.get_frame_hessian(robot.model.getFrameId(frame), robot.get_joint_v_offset(joint));
          })
      .def("get_T_world_fbase", &RobotType::get_T_world_fbase)
      .def("set_T_world_fbase", &RobotType::set_T_world_fbase)
      .def("com_world", &RobotType::com_world)
      .def("centroidal_map", &RobotType::centroidal_map)
      .def("joint_names", &RobotType::joint_names, joint_names_overloads())
      .def("frame_names", &RobotType::frame_names)
      .def("self_collisions", &RobotType::self_collisions)
      .def("distances", &RobotType::distances)
      .def("com_jacobian", &RobotType::com_jacobian)
      .def("com_jacobian_time_variation", &RobotType::com_jacobian_time_variation)
      .def("generalized_gravity", &RobotType::generalized_gravity)
      .def("non_linear_effects", &RobotType::non_linear_effects)
      .def("set_rotor_inertia", &RobotType::set_rotor_inertia)
      .def("set_gear_ratio", &RobotType::set_gear_ratio)
      .def("mass_matrix", &RobotType::mass_matrix)
      .def("set_gravity", &RobotType::set_gravity)
      .def("total_mass", &RobotType::total_mass)
      .def("integrate", &RobotType::integrate)
      .def(
          "static_gravity_compensation_torques",
          +[](RobotType& robot, const std::string& frame) {
    return robot.static_gravity_compensation_torques(frame); })
      .def(
          "static_gravity_compensation_torques_dict",
          +[](RobotType& robot, const std::string& frame) {
    auto torques = robot.static_gravity_compensation_torques(frame);
    boost::python::dict dict;

    for (auto& dof : robot.joint_names())
    {
      dict[dof] = torques[robot.get_joint_v_offset(dof)];
    }

    return dict;
          })
      .def(
          "torques_from_acceleration_with_fixed_frame",
          +[](RobotType& robot, Eigen::VectorXd qdd_a, const std::string& frame) {
    return robot.torques_from_acceleration_with_fixed_frame(qdd_a, frame);
          })
      .def(
          "torques_from_acceleration_with_fixed_frame_dict",
          +[](RobotType& robot, Eigen::VectorXd qdd_a, const std::string& frame) {
    auto torques = robot.torques_from_acceleration_with_fixed_frame(qdd_a, frame);
    boost::python::dict dict;

    for (auto& dof : robot.joint_names())
    {
      dict[dof] = torques[robot.get_joint_v_offset(dof) - 6];
    }

    return dict;
          })
      .def(
          "get_T_world_frame",
          +[](RobotType& robot, const std::string& frame) {
    return robot.get_T_world_frame(frame); })
      .def(
          "get_T_a_b", +[](RobotType& robot, const std::string& frameA,
                           const std::string& frameB) {
    return robot.get_T_a_b(frameA, frameB); })
      .def(
          "set_T_world_frame", +[](RobotType& robot, const std::string& frame,
                                   Eigen::Affine3d T_world_frame) {
    robot.set_T_world_frame(frame, T_world_frame); })
      .def(
          "frame_jacobian", +[](RobotType& robot, const std::string& frame,
                                const std::string& reference) {
    return robot.frame_jacobian(frame, reference); })
      .def(
          "frame_jacobian_time_variation",
          +[](RobotType& robot, const std::string& frame, const std::string& reference) {
    return robot.frame_jacobian_time_variation(frame, reference);
          })
      .def(
          "relative_position_jacobian",
          +[](RobotType& robot, const std::string& frameA, const std::string& frameB) {
    return robot.relative_position_jacobian(frameA, frameB);
          })
      .def(
          "joint_jacobian", +[](RobotType& robot, const std::string& joint,
                                const std::string& reference) {
    return robot.joint_jacobian(joint, reference); })
      .def(
          "make_solver", +[](RobotType& robot) {
    return placo::kinematics::KinematicsSolver(robot); })
      .def("add_q_noise", &RobotType::add_q_noise);
}

void exposeRobotWrapper()
{
  enum_<RobotWrapper::Flags>("Flags")
      .value("collision_as_visual", RobotWrapper::Flags::COLLISION_AS_VISUAL)
      .value("ignore_collisions", RobotWrapper::Flags::IGNORE_COLLISIONS);

  class__<RobotWrapper::State>("RobotWrapper_State")
      .def_readwrite("q", &RobotWrapper::State::q, np_docstring("Robot joints configuration"))
      .def_readwrite("qd", &RobotWrapper::State::qd, np_docstring("Robot joints velocity"))
      .def_readwrite("qdd", &RobotWrapper::State::qdd, np_docstring("Robot joints acceleration"));

  class__<RobotWrapper::Collision>("Collision")
      .add_property("objA", &RobotWrapper::Collision::objA, typed_docstring<int>("Object A"))
      .add_property("objB", &RobotWrapper::Collision::objB, typed_docstring<int>("Object B"))
      .add_property("bodyA", &RobotWrapper::Collision::bodyA, typed_docstring<std::string>("Body A"))
      .add_property("bodyB", &RobotWrapper::Collision::bodyB, typed_docstring<std::string>("Body A"))
      .add_property("parentA", &RobotWrapper::Collision::parentA, typed_docstring<pinocchio::JointIndex>("Parent A"))
      .add_property("parentB", &RobotWrapper::Collision::parentB, typed_docstring<pinocchio::JointIndex>("Parent B"))
      .def(
          "get_contact", +[](RobotWrapper::Collision& collision, int index) { return collision.contacts[index]; },
          args("self", "index"), "Get the `intex`th contact");

  class__<RobotWrapper::Distance>("Distance")
      .add_property("objA", &RobotWrapper::Distance::objA, typed_docstring<int>("Object A"))
      .add_property("objB", &RobotWrapper::Distance::objB, typed_docstring<int>("Object B"))
      .add_property("parentA", &RobotWrapper::Distance::parentA, typed_docstring<pinocchio::JointIndex>("Parent A"))
      .add_property("parentB", &RobotWrapper::Distance::parentB, typed_docstring<pinocchio::JointIndex>("Parent B"))
      .def_readwrite("pointA", &RobotWrapper::Distance::pointA, np_docstring("Position on A"))
      .def_readwrite("pointB", &RobotWrapper::Distance::pointB, np_docstring("Position on B"))
      .add_property("min_distance", &RobotWrapper::Distance::min_distance, typed_docstring<float>("Minimum distance"));

  class_<RobotWrapper> robotWrapper =
      class__<RobotWrapper>("RobotWrapper", init<std::string, optional<int, std::string>>());
  exposeRobotType<RobotWrapper>(robotWrapper);

  class_<HumanoidRobot, bases<RobotWrapper>> humanoidWrapper =
      class__<HumanoidRobot, bases<RobotWrapper>>("HumanoidRobot", init<std::string, optional<int, std::string>>());

  exposeRobotType<HumanoidRobot>(humanoidWrapper);
  humanoidWrapper
      .def<void (HumanoidRobot::*)(const std::string&)>("update_support_side", &HumanoidRobot::update_support_side)
      .def("ensure_on_floor", &HumanoidRobot::ensure_on_floor)
      .def("get_T_world_left", &HumanoidRobot::get_T_world_left)
      .def("get_T_world_right", &HumanoidRobot::get_T_world_right)
      .def("get_T_world_trunk", &HumanoidRobot::get_T_world_trunk)
      .def("get_com_velocity", &HumanoidRobot::get_com_velocity)
      .def("dcm", &HumanoidRobot::dcm)
      .def("zmp", &HumanoidRobot::zmp)
      .def("other_side", &HumanoidRobot::other_side)
      .def(
          "get_torques", +[](HumanoidRobot& robot, Eigen::VectorXd qdd_a, Eigen::VectorXd contact_forces,
                             bool use_nle) { return robot.get_torques(qdd_a, contact_forces, use_nle); })
      .def(
          "get_torques_dict",
          +[](HumanoidRobot& robot, Eigen::VectorXd qdd_a, Eigen::VectorXd contact_forces, bool use_nle) {
            auto torques = robot.get_torques(qdd_a, contact_forces, use_nle);
            boost::python::dict dict;

            for (auto& dof : robot.joint_names())
            {
              dict[dof] = torques[robot.get_joint_v_offset(dof)];
            }

            return dict;
          })
#ifdef HAVE_RHOBAN_UTILS
      .def("read_from_histories", &HumanoidRobot::read_from_histories, read_from_histories_overloads())
#endif
      .def(
          "get_support_side", +[](const HumanoidRobot& robot) { return robot.support_side; })
      .add_property("support_is_both", &HumanoidRobot::support_is_both, &HumanoidRobot::support_is_both)
      .def_readwrite("T_world_support", &HumanoidRobot::T_world_support);

  exposeStdVector<RobotWrapper::Collision>("vector_Collision");
  exposeStdVector<RobotWrapper::Distance>("vector_Distance");
}