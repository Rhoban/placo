#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/control/kinematics_solver.h"
#include <boost/python/return_internal_reference.hpp>
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

class_<KinematicsSolver>* solver_class_ptr = nullptr;

template <typename T>
void registerTaskMethods(class_<T>& class__)
{
  class__.add_property("name", &T::name)
      .add_property("solver", &T::solver)
      .add_property("weight", &T::weight)
      .add_property(
          "priority", +[](const T& task) { return (task.priority == Task::Soft) ? "soft" : "hard"; })
      .add_property("A", &T::A)
      .add_property("b", &T::b)
      .def("error", &T::error)
      .def("update", &T::update)
      .def("set_priority", &T::set_priority)
      .def("set_weight", &T::set_weight)
      .def("set_name", &T::set_name)
      .def(
          "configure", +[](T& task, std::string name, std::string priority, double weight) {
            task.configure(name, priority, weight);
          });

  solver_class_ptr->def(
      "remove_task", +[](KinematicsSolver& solver, T& task) { solver.remove_task(&task); });
}

void exposeKinematics()
{
  class_<KinematicsSolver> solver_class =
      class_<KinematicsSolver>("KinematicsSolver", init<RobotWrapper&>())
          .add_property("noise", &KinematicsSolver::noise, &KinematicsSolver::noise)
          .add_property("dt", &KinematicsSolver::dt, &KinematicsSolver::dt)
          .add_property("N", &KinematicsSolver::N)
          .add_property(
              "robot",
              +[](const KinematicsSolver& solver) {
                RobotWrapper& wrapper = *solver.robot;
                return wrapper;
              })

          // Position and CoM task
          .def<PositionTask& (KinematicsSolver::*)(std::string, Eigen::Vector3d)>(
              "add_position_task", &KinematicsSolver::add_position_task, return_internal_reference<>())
          .def<RelativePositionTask& (KinematicsSolver::*)(std::string, std::string, Eigen::Vector3d)>(
              "add_relative_position_task", &KinematicsSolver::add_relative_position_task,
              return_internal_reference<>())
          .def<CoMTask& (KinematicsSolver::*)(Eigen::Vector3d)>("add_com_task", &KinematicsSolver::add_com_task,
                                                                return_internal_reference<>())

          // Orientation task
          .def<OrientationTask& (KinematicsSolver::*)(std::string, Eigen::Matrix3d)>(
              "add_orientation_task", &KinematicsSolver::add_orientation_task, return_internal_reference<>())
          .def<RelativeOrientationTask& (KinematicsSolver::*)(std::string, std::string, Eigen::Matrix3d)>(
              "add_relative_orientation_task", &KinematicsSolver::add_relative_orientation_task,
              return_internal_reference<>())

          // Axis tasks
          .def<AxisAlignTask& (KinematicsSolver::*)(std::string, Eigen::Vector3d, Eigen::Vector3d)>(
              "add_axisalign_task", &KinematicsSolver::add_axisalign_task, return_internal_reference<>())
          .def<AxisPlaneTask& (KinematicsSolver::*)(std::string, Eigen::Vector3d, Eigen::Vector3d)>(
              "add_axisplane_task", &KinematicsSolver::add_axisplane_task, return_internal_reference<>())

          // Frame task
          .def<FrameTask (KinematicsSolver::*)(std::string, Eigen::Affine3d)>("add_frame_task",
                                                                              &KinematicsSolver::add_frame_task)
          .def<RelativeFrameTask (KinematicsSolver::*)(std::string, std::string, Eigen::Affine3d)>(
              "add_relative_frame_task", &KinematicsSolver::add_relative_frame_task)

          // Pose task
          .def<PoseTask& (KinematicsSolver::*)(std::string, Eigen::Affine3d)>(
              "add_pose_task", &KinematicsSolver::add_pose_task, return_internal_reference<>())
          .def<RelativePoseTask& (KinematicsSolver::*)(std::string, std::string, Eigen::Affine3d)>(
              "add_relative_pose_task", &KinematicsSolver::add_relative_pose_task, return_internal_reference<>())

          // Joint task
          .def<JointTask& (KinematicsSolver::*)(std::string, double)>(
              "add_joint_task", &KinematicsSolver::add_joint_task, return_internal_reference<>())
          .def<JointsTask& (KinematicsSolver::*)(void)>("add_joints_task", &KinematicsSolver::add_joints_task,
                                                        return_internal_reference<>())

          // Distance task
          .def<DistanceTask& (KinematicsSolver::*)(std::string, std::string, double)>(
              "add_distance_task", &KinematicsSolver::add_distance_task, return_internal_reference<>())

          // Centroidal Momentum Task
          .def<CentroidalMomentumTask& (KinematicsSolver::*)(Eigen::Vector3d)>(
              "add_centroidal_momentum_task", &KinematicsSolver::add_centroidal_momentum_task,
              return_internal_reference<>())

          // Regularization task
          .def("add_regularization_task", &KinematicsSolver::add_regularization_task, return_internal_reference<>())

          // Masking/unmasking DoFs
          .def("mask_dof", &KinematicsSolver::mask_dof)
          .def("unmask_dof", &KinematicsSolver::unmask_dof)
          .def("mask_fbase", &KinematicsSolver::mask_fbase)

          // Clearing the tasks
          .def("clear_tasks", &KinematicsSolver::clear_tasks)
          .def("dump_status", &KinematicsSolver::dump_status)

          .def("tasks_count", &KinematicsSolver::tasks_count)

          .def("enable_joint_limits", &KinematicsSolver::enable_joint_limits)
          .def("enable_velocity_limits", &KinematicsSolver::enable_velocity_limits)
          .def("enable_velocity_post_limits", &KinematicsSolver::enable_velocity_post_limits)
          .def("enable_self_collision_avoidance", &KinematicsSolver::enable_self_collision_avoidance)

          .def(
              "remove_task", +[](KinematicsSolver& solver, FrameTask& task) { solver.remove_task(task); })

          .def("solve", &KinematicsSolver::solve);

  solver_class_ptr = &solver_class;

  registerTaskMethods(class_<PositionTask>("PositionTask", init<RobotWrapper::FrameIndex, Eigen::Vector3d>())
                          .add_property("frame_index", &PositionTask::frame_index)
                          .add_property(
                              "target_world", +[](const PositionTask& task) { return task.target_world; },
                              &PositionTask::target_world));

  registerTaskMethods(
      class_<RelativePositionTask>("RelativePositionTask",
                                   init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Vector3d>())
          .add_property("frame_a", &RelativePositionTask::frame_a)
          .add_property("frame_b", &RelativePositionTask::frame_b)
          .add_property(
              "target", +[](const RelativePositionTask& task) { return task.target; }, &RelativePositionTask::target));

  registerTaskMethods(class_<CoMTask>("CoMTask", init<Eigen::Vector3d>())
                          .add_property("target_world", &CoMTask::target_world, &CoMTask::target_world));

  registerTaskMethods(class_<OrientationTask>("OrientationTask", init<RobotWrapper::FrameIndex, Eigen::Matrix3d>())
                          .add_property("frame_index", &OrientationTask::frame_index)
                          .add_property(
                              "R_world_frame", +[](const OrientationTask& task) { return task.R_world_frame; },
                              &OrientationTask::R_world_frame));

  registerTaskMethods(
      class_<RelativeOrientationTask>("RelativeOrientationTask",
                                      init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Matrix3d>())
          .add_property("frame_a", &RelativeOrientationTask::frame_a)
          .add_property("frame_b", &RelativeOrientationTask::frame_b)
          .add_property(
              "R_a_b", +[](const RelativeOrientationTask& task) { return task.R_a_b; },
              &RelativeOrientationTask::R_a_b));

  class_<FrameTask>("FrameTask", init<>())
      .def(
          "position", +[](const FrameTask& task) -> PositionTask& { return *task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const FrameTask& task) -> OrientationTask& { return *task.orientation; },
          return_internal_reference<>())
      .def("configure", &FrameTask::configure)
      .add_property("T_world_frame", &FrameTask::get_T_world_frame, &FrameTask::set_T_world_frame);

  class_<RelativeFrameTask>("RelativeFrameTask", init<RelativePositionTask&, RelativeOrientationTask&>())
      .def(
          "position", +[](const RelativeFrameTask& task) -> RelativePositionTask& { return task.position; },
          return_internal_reference<>())

      .def(
          "orientation", +[](const RelativeFrameTask& task) -> RelativeOrientationTask& { return task.orientation; },
          return_internal_reference<>())
      .def("configure", &RelativeFrameTask::configure)
      .add_property("T_a_b", &RelativeFrameTask::get_T_a_b, &RelativeFrameTask::set_T_a_b);

  registerTaskMethods(
      class_<AxisAlignTask>("AxisAlignTask", init<RobotWrapper::FrameIndex, Eigen::Vector3d, Eigen::Vector3d>())
          .add_property("frame_index", &AxisAlignTask::frame_index)
          .add_property(
              "axis_frame", +[](const AxisAlignTask& task) { return task.axis_frame; }, &AxisAlignTask::axis_frame)
          .add_property(
              "targetAxis_world", +[](const AxisAlignTask& task) { return task.targetAxis_world; },
              &AxisAlignTask::targetAxis_world));

  registerTaskMethods(
      class_<AxisPlaneTask>("AxisPlaneTask", init<RobotWrapper::FrameIndex, Eigen::Vector3d, Eigen::Vector3d>())
          .add_property("frame_index", &AxisPlaneTask::frame_index)
          .add_property(
              "axis_frame", +[](const AxisPlaneTask& task) { return task.axis_frame; }, &AxisPlaneTask::axis_frame)
          .add_property(
              "normal_world", +[](const AxisPlaneTask& task) { return task.normal_world; },
              &AxisPlaneTask::normal_world));

  registerTaskMethods(
      class_<PoseTask>("PoseTask", init<RobotWrapper::FrameIndex, Eigen::Affine3d>())
          .add_property("frame_index", &PoseTask::frame_index)
          .add_property(
              "T_world_frame", +[](const PoseTask& task) { return task.T_world_frame; }, &PoseTask::T_world_frame));

  registerTaskMethods(
      class_<RelativePoseTask>("RelativePoseTask",
                               init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, Eigen::Affine3d>())
          .add_property("frame_a", &RelativePoseTask::frame_a)
          .add_property("frame_b", &RelativePoseTask::frame_b)
          .add_property(
              "T_a_b", +[](const RelativePoseTask& task) { return task.T_a_b; }, &RelativePoseTask::T_a_b));

  registerTaskMethods(class_<JointTask>("JointTask", init<std::string, double>())
                          .add_property("joint", &JointTask::joint)
                          .add_property("target", &JointTask::target, &JointTask::target));

  registerTaskMethods(class_<JointsTask>("JointsTask", init<>())
                          .def("set_joint", &JointsTask::set_joint)
                          .def(
                              "set_joints", +[](JointsTask& task, boost::python::dict& py_dict) {
                                update_map<std::string, double>(task.joints, py_dict);
                              }));

  registerTaskMethods(
      class_<DistanceTask>("DistanceTask", init<RobotWrapper::FrameIndex, RobotWrapper::FrameIndex, double>())
          .add_property("frame_a", &DistanceTask::frame_a)
          .add_property("frame_b", &DistanceTask::frame_b)
          .add_property("distance", &DistanceTask::distance, &DistanceTask::distance));

  registerTaskMethods(class_<CentroidalMomentumTask>("CentroidalMomentumTask", init<Eigen::Vector3d>())
                          .def("mask_axis", &CentroidalMomentumTask::mask_axis)
                          .add_property("L_world", &CentroidalMomentumTask::L_world, &CentroidalMomentumTask::L_world));

  auto regularizationTask = class_<RegularizationTask>("RegularizationTask");
  registerTaskMethods(regularizationTask);
}