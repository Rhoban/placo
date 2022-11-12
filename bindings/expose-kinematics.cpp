#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/control/kinematics_solver.h"
#include <boost/python/return_internal_reference.hpp>
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

template <typename T>
void registerTaskMethods(class_<T>& class__)
{
  class__.add_property("name", &T::name)
      .add_property("solver", &T::solver)
      .add_property("weight", &T::weight)
      .add_property(
          "priority", +[](const T& task) { return (task.priority == KinematicsSolver::Soft) ? "soft" : "hard"; })
      .add_property("A", &T::A)
      .add_property("b", &T::b)
      .def("error", &T::error)
      .def("update", &T::update)
      .def("set_priority", &T::set_priority)
      .def("set_weight", &T::set_weight)
      .def("set_name", &T::set_name)
      .def("configure", &T::configure);
}

void exposeKinematics()
{
  registerTaskMethods(
      class_<KinematicsSolver::PositionTask>("PositionTask", init<MobileRobot::FrameIndex, Eigen::Vector3d>())
          .add_property("frame_index", &KinematicsSolver::PositionTask::frame_index)
          .add_property(
              "target_world", +[](const KinematicsSolver::PositionTask& task) { return task.target_world; },
              &KinematicsSolver::PositionTask::target_world));

  registerTaskMethods(
      class_<KinematicsSolver::RelativePositionTask>(
          "RelativePositionTask", init<MobileRobot::FrameIndex, MobileRobot::FrameIndex, Eigen::Vector3d>())
          .add_property("frame_a", &KinematicsSolver::RelativePositionTask::frame_a)
          .add_property("frame_b", &KinematicsSolver::RelativePositionTask::frame_b)
          .add_property(
              "target", +[](const KinematicsSolver::RelativePositionTask& task) { return task.target; },
              &KinematicsSolver::RelativePositionTask::target));

  registerTaskMethods(class_<KinematicsSolver::CoMTask>("CoMTask", init<Eigen::Vector3d>())
                          .add_property("target_world", &KinematicsSolver::CoMTask::target_world,
                                        &KinematicsSolver::CoMTask::target_world));

  registerTaskMethods(
      class_<KinematicsSolver::OrientationTask>("OrientationTask", init<MobileRobot::FrameIndex, Eigen::Matrix3d>())
          .add_property("frame_index", &KinematicsSolver::OrientationTask::frame_index)
          .add_property(
              "R_world_frame", +[](const KinematicsSolver::OrientationTask& task) { return task.R_world_frame; },
              &KinematicsSolver::OrientationTask::R_world_frame));

  registerTaskMethods(
      class_<KinematicsSolver::RelativeOrientationTask>(
          "RelativeOrientationTask", init<MobileRobot::FrameIndex, MobileRobot::FrameIndex, Eigen::Matrix3d>())
          .add_property("frame_a", &KinematicsSolver::RelativeOrientationTask::frame_a)
          .add_property("frame_b", &KinematicsSolver::RelativeOrientationTask::frame_b)
          .add_property(
              "R_a_b", +[](const KinematicsSolver::RelativeOrientationTask& task) { return task.R_a_b; },
              &KinematicsSolver::RelativeOrientationTask::R_a_b));

  class_<KinematicsSolver::FrameTask>("FrameTask",
                                      init<KinematicsSolver::PositionTask&, KinematicsSolver::OrientationTask&>())
      .def(
          "position",
          +[](const KinematicsSolver::FrameTask& task) -> KinematicsSolver::PositionTask& { return task.position; },
          return_internal_reference<>())

      .def(
          "orientation",
          +[](const KinematicsSolver::FrameTask& task) -> KinematicsSolver::OrientationTask& {
            return task.orientation;
          },
          return_internal_reference<>())
      .def("configure", &KinematicsSolver::FrameTask::configure)
      .add_property("T_world_frame", &KinematicsSolver::FrameTask::get_T_world_frame,
                    &KinematicsSolver::FrameTask::set_T_world_frame);

  class_<KinematicsSolver::RelativeFrameTask>(
      "RelativeFrameTask", init<KinematicsSolver::RelativePositionTask&, KinematicsSolver::RelativeOrientationTask&>())
      .def(
          "position",
          +[](const KinematicsSolver::RelativeFrameTask& task) -> KinematicsSolver::RelativePositionTask& {
            return task.position;
          },
          return_internal_reference<>())

      .def(
          "orientation",
          +[](const KinematicsSolver::RelativeFrameTask& task) -> KinematicsSolver::RelativeOrientationTask& {
            return task.orientation;
          },
          return_internal_reference<>())
      .def("configure", &KinematicsSolver::RelativeFrameTask::configure)
      .add_property("T_a_b", &KinematicsSolver::RelativeFrameTask::get_T_a_b,
                    &KinematicsSolver::RelativeFrameTask::set_T_a_b);

  registerTaskMethods(
      class_<KinematicsSolver::AxisAlignTask>("AxisAlignTask",
                                              init<MobileRobot::FrameIndex, Eigen::Vector3d, Eigen::Vector3d>())
          .add_property("frame_index", &KinematicsSolver::AxisAlignTask::frame_index)
          .add_property(
              "axis_frame", +[](const KinematicsSolver::AxisAlignTask& task) { return task.axis_frame; },
              &KinematicsSolver::AxisAlignTask::axis_frame)
          .add_property(
              "targetAxis_world", +[](const KinematicsSolver::AxisAlignTask& task) { return task.targetAxis_world; },
              &KinematicsSolver::AxisAlignTask::targetAxis_world));

  registerTaskMethods(class_<KinematicsSolver::PoseTask>("PoseTask", init<MobileRobot::FrameIndex, Eigen::Affine3d>())
                          .add_property("frame_index", &KinematicsSolver::PoseTask::frame_index)
                          .add_property(
                              "T_world_frame",
                              +[](const KinematicsSolver::PoseTask& task) { return task.T_world_frame; },
                              &KinematicsSolver::PoseTask::T_world_frame));

  registerTaskMethods(class_<KinematicsSolver::RelativePoseTask>(
                          "RelativePoseTask", init<MobileRobot::FrameIndex, MobileRobot::FrameIndex, Eigen::Affine3d>())
                          .add_property("frame_a", &KinematicsSolver::RelativePoseTask::frame_a)
                          .add_property("frame_b", &KinematicsSolver::RelativePoseTask::frame_b)
                          .add_property(
                              "T_a_b", +[](const KinematicsSolver::RelativePoseTask& task) { return task.T_a_b; },
                              &KinematicsSolver::RelativePoseTask::T_a_b));

  registerTaskMethods(
      class_<KinematicsSolver::JointTask>("JointTask", init<std::string, double>())
          .add_property("joint", &KinematicsSolver::JointTask::joint)
          .add_property("target", &KinematicsSolver::JointTask::target, &KinematicsSolver::JointTask::target));

  registerTaskMethods(class_<KinematicsSolver::JointsTask>("JointsTask", init<>())
                          .def("set_joint", &KinematicsSolver::JointsTask::set_joint)
                          .def(
                              "set_joints", +[](KinematicsSolver::JointsTask& task, boost::python::dict& py_dict) {
                                update_map<std::string, double>(task.joints, py_dict);
                              }));

  auto regularizationTask = class_<KinematicsSolver::RegularizationTask>("RegularizationTask");
  registerTaskMethods(regularizationTask);

  class_<KinematicsSolver>("KinematicsSolver", init<MobileRobot&>())
      // Position and CoM task
      .def<KinematicsSolver::PositionTask& (KinematicsSolver::*)(std::string, Eigen::Vector3d)>(
          "add_position_task", &KinematicsSolver::add_position_task, return_internal_reference<>())
      .def<KinematicsSolver::RelativePositionTask& (KinematicsSolver::*)(std::string, std::string, Eigen::Vector3d)>(
          "add_relative_position_task", &KinematicsSolver::add_relative_position_task, return_internal_reference<>())
      .def<KinematicsSolver::CoMTask& (KinematicsSolver::*)(Eigen::Vector3d)>(
          "add_com_task", &KinematicsSolver::add_com_task, return_internal_reference<>())

      // Orientation task
      .def<KinematicsSolver::OrientationTask& (KinematicsSolver::*)(std::string, Eigen::Matrix3d)>(
          "add_orientation_task", &KinematicsSolver::add_orientation_task, return_internal_reference<>())
      .def<KinematicsSolver::RelativeOrientationTask& (KinematicsSolver::*)(std::string, std::string, Eigen::Matrix3d)>(
          "add_relative_orientation_task", &KinematicsSolver::add_relative_orientation_task,
          return_internal_reference<>())

      // Axis align task
      .def<KinematicsSolver::AxisAlignTask& (KinematicsSolver::*)(std::string, Eigen::Vector3d, Eigen::Vector3d)>(
          "add_axisalign_task", &KinematicsSolver::add_axisalign_task, return_internal_reference<>())

      // Frame task
      .def<KinematicsSolver::FrameTask (KinematicsSolver::*)(std::string, Eigen::Affine3d)>(
          "add_frame_task", &KinematicsSolver::add_frame_task)
      .def<KinematicsSolver::RelativeFrameTask (KinematicsSolver::*)(std::string, std::string, Eigen::Affine3d)>(
          "add_relative_frame_task", &KinematicsSolver::add_relative_frame_task)

      // Pose task
      .def<KinematicsSolver::PoseTask& (KinematicsSolver::*)(std::string, Eigen::Affine3d)>(
          "add_pose_task", &KinematicsSolver::add_pose_task, return_internal_reference<>())
      .def<KinematicsSolver::RelativePoseTask& (KinematicsSolver::*)(std::string, std::string, Eigen::Affine3d)>(
          "add_relative_pose_task", &KinematicsSolver::add_relative_pose_task, return_internal_reference<>())

      // Joint task
      .def<KinematicsSolver::JointTask& (KinematicsSolver::*)(std::string, double)>(
          "add_joint_task", &KinematicsSolver::add_joint_task, return_internal_reference<>())
      .def<KinematicsSolver::JointsTask& (KinematicsSolver::*)(void)>(
          "add_joints_task", &KinematicsSolver::add_joints_task, return_internal_reference<>())

      // Regularization task
      .def("add_regularization_task", &KinematicsSolver::add_regularization_task, return_internal_reference<>())

      // Masking/unmasking DoFs
      .def("mask_dof", &KinematicsSolver::mask_dof)
      .def("unmask_dof", &KinematicsSolver::unmask_dof)

      // Clearing the tasks
      .def("clear_tasks", &KinematicsSolver::clear_tasks)
      .def("dump_status", &KinematicsSolver::dump_status)

      .def("solve", &KinematicsSolver::solve);
}