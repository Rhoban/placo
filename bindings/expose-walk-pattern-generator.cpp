#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "doxystub.h"
#include "placo/humanoid/walk_pattern_generator.h"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/humanoid/footsteps_planner.h"
#include "placo/humanoid/swing_foot.h"
#include "placo/humanoid/swing_foot_quintic.h"
#include "placo/humanoid/swing_foot_cubic.h"
#include "placo/humanoid/walk_tasks.h"
#include "placo/humanoid/lipm.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigen-to-python.hpp>

using namespace placo;
using namespace boost::python;
using namespace placo::kinematics;
using namespace placo::humanoid;

void exposeWalkPatternGenerator()
{
  class__<WalkPatternGenerator::TrajectoryPart>("WPGTrajectoryPart", init<FootstepsPlanner::Support, double>())
      .add_property("t_start", &WalkPatternGenerator::TrajectoryPart::t_start,
                    &WalkPatternGenerator::TrajectoryPart::t_start)
      .add_property("t_end", &WalkPatternGenerator::TrajectoryPart::t_end, &WalkPatternGenerator::TrajectoryPart::t_end)
      .add_property("support", &WalkPatternGenerator::TrajectoryPart::support,
                    &WalkPatternGenerator::TrajectoryPart::support);

  class__<WalkPatternGenerator::Trajectory>("WPGTrajectory", init<double, double, double, double>())
      .add_property("t_start", &WalkPatternGenerator::Trajectory::t_start)
      .add_property("t_end", &WalkPatternGenerator::Trajectory::t_end)
      .add_property("com_target_z", &WalkPatternGenerator::Trajectory::com_target_z)
      .add_property("trunk_pitch", &WalkPatternGenerator::Trajectory::trunk_pitch)
      .add_property("trunk_roll", &WalkPatternGenerator::Trajectory::trunk_roll)
      .add_property("kept_ts", &WalkPatternGenerator::Trajectory::kept_ts)
      .add_property("parts", &WalkPatternGenerator::Trajectory::parts)
      .def("get_T_world_left", &WalkPatternGenerator::Trajectory::get_T_world_left)
      .def("get_T_world_right", &WalkPatternGenerator::Trajectory::get_T_world_right)
      .def("get_v_world_right", &WalkPatternGenerator::Trajectory::get_v_world_right)
      .def("get_v_world_foot", &WalkPatternGenerator::Trajectory::get_v_world_foot)
      .def("get_p_world_CoM", &WalkPatternGenerator::Trajectory::get_p_world_CoM)
      .def("get_v_world_CoM", &WalkPatternGenerator::Trajectory::get_v_world_CoM)
      .def("get_a_world_CoM", &WalkPatternGenerator::Trajectory::get_a_world_CoM)
      .def("get_j_world_CoM", &WalkPatternGenerator::Trajectory::get_j_world_CoM)
      .def("get_p_world_ZMP", &WalkPatternGenerator::Trajectory::get_p_world_ZMP)
      .def("get_p_world_DCM", &WalkPatternGenerator::Trajectory::get_p_world_DCM)
      .def("get_R_world_trunk", &WalkPatternGenerator::Trajectory::get_R_world_trunk)
      .def("get_p_support_CoM", &WalkPatternGenerator::Trajectory::get_p_support_CoM)
      .def("get_v_support_CoM", &WalkPatternGenerator::Trajectory::get_v_support_CoM)
      .def("get_p_support_DCM", &WalkPatternGenerator::Trajectory::get_p_support_DCM)
      .def("support_side", &WalkPatternGenerator::Trajectory::support_side)
      .def("support_is_both", &WalkPatternGenerator::Trajectory::support_is_both)
      .def("get_supports", &WalkPatternGenerator::Trajectory::get_supports)
      .def("get_support", &WalkPatternGenerator::Trajectory::get_support)
      .def("get_next_support", &WalkPatternGenerator::Trajectory::get_next_support)
      .def("get_prev_support", &WalkPatternGenerator::Trajectory::get_prev_support)
      .def("get_part_t_end", &WalkPatternGenerator::Trajectory::get_part_t_end)
      .def("get_part_t_start", &WalkPatternGenerator::Trajectory::get_part_t_start)
      .def("get_part_end_dcm", &WalkPatternGenerator::Trajectory::get_part_end_dcm)
      .def("apply_transform", &WalkPatternGenerator::Trajectory::apply_transform)
      .def("print_parts_timings", &WalkPatternGenerator::Trajectory::print_parts_timings);

  class__<WalkPatternGenerator>("WalkPatternGenerator", init<HumanoidRobot&, HumanoidParameters&>())
      .def("plan", &WalkPatternGenerator::plan)
      .def("replan", &WalkPatternGenerator::replan)
      .def("can_replan_supports", &WalkPatternGenerator::can_replan_supports)
      .def("replan_supports", &WalkPatternGenerator::replan_supports)
      .def("update_supports", &WalkPatternGenerator::update_supports)
      .def("get_optimal_zmp", &WalkPatternGenerator::get_optimal_zmp)
      .def("support_default_timesteps", &WalkPatternGenerator::support_default_timesteps)
      .def("support_default_duration", &WalkPatternGenerator::support_default_duration)
      .add_property("soft", &WalkPatternGenerator::soft, &WalkPatternGenerator::soft)
      .add_property("zmp_in_support_weight", &WalkPatternGenerator::zmp_in_support_weight,
                    &WalkPatternGenerator::zmp_in_support_weight)
      .add_property("stop_end_support_weight", &WalkPatternGenerator::stop_end_support_weight,
                    &WalkPatternGenerator::stop_end_support_weight);

  class__<SwingFoot>("SwingFoot", init<>())
      .def("make_trajectory", &SwingFoot::make_trajectory)
      .def("remake_trajectory", &SwingFoot::remake_trajectory);

  class__<SwingFootCubic::Trajectory>("SwingFootCubicTrajectory", init<>())
      .def("pos", &SwingFootCubic::Trajectory::pos)
      .def("vel", &SwingFootCubic::Trajectory::vel);

  class__<SwingFootCubic>("SwingFootCubic", init<>()).def("make_trajectory", &SwingFootCubic::make_trajectory);

  class__<SwingFoot::Trajectory>("SwingFootTrajectory", init<>())
      .def("pos", &SwingFoot::Trajectory::pos)
      .def("vel", &SwingFoot::Trajectory::vel);

  class__<SwingFootQuintic>("SwingFootQuintic", init<>()).def("make_trajectory", &SwingFootQuintic::make_trajectory);

  class__<SwingFootQuintic::Trajectory>("SwingFootQuinticTrajectory", init<>())
      .def("pos", &SwingFootQuintic::Trajectory::pos)
      .def("vel", &SwingFootQuintic::Trajectory::vel);

  class__<WalkTasks>("WalkTasks", init<>())
      .def(
          "initialize_tasks", +[](WalkTasks& tasks, KinematicsSolver& solver,
                                  HumanoidRobot& robot) { tasks.initialize_tasks(&solver, &robot); })
      .def(
          "update_tasks_from_trajectory", +[](WalkTasks& tasks, WalkPatternGenerator::Trajectory& trajectory,
                                              double t) { return tasks.update_tasks(trajectory, t); })
      .def(
          "update_tasks",
          +[](WalkTasks& tasks, Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right, Eigen::Vector3d com_world,
              Eigen::Matrix3d R_world_trunk) {
            return tasks.update_tasks(T_world_left, T_world_right, com_world, R_world_trunk);
          })
      .def(
          "reach_initial_pose",
          +[](WalkTasks& tasks, Eigen::Affine3d T_world_left, double feet_spacing, double com_height,
              double trunk_pitch) {
            return tasks.reach_initial_pose(T_world_left, feet_spacing, com_height, trunk_pitch);
          })
      .def("remove_tasks", &WalkTasks::remove_tasks)
      .def(
          "get_tasks_error",
          +[](WalkTasks& tasks) {
            auto errors = tasks.get_tasks_error();
            boost::python::dict dict;
            for (auto key : errors)
            {
              dict[key.first + "_x"] = key.second[0];
              dict[key.first + "_y"] = key.second[1];
              dict[key.first + "_z"] = key.second[2];
            }
            return dict;
          })
      .add_property(
          "solver", +[](WalkTasks& tasks) { return *tasks.solver; })
      .add_property("scaled", &WalkTasks::scaled, &WalkTasks::scaled)
      .add_property("left_foot_task", &WalkTasks::left_foot_task)
      .add_property("right_foot_task", &WalkTasks::right_foot_task)
      .add_property("trunk_mode", &WalkTasks::trunk_mode, &WalkTasks::trunk_mode)
      .add_property("com_x", &WalkTasks::com_x, &WalkTasks::com_x)
      .add_property("com_y", &WalkTasks::com_y, &WalkTasks::com_y)
      .add_property("trunk_orientation_task",
                    make_function(
                        +[](WalkTasks& tasks) -> OrientationTask& { return *tasks.trunk_orientation_task; },
                        return_value_policy<reference_existing_object>()))
      .add_property("com_task", make_function(
                                    +[](WalkTasks& tasks) -> CoMTask& { return *tasks.com_task; },
                                    return_value_policy<reference_existing_object>()))
      .add_property("trunk_task", make_function(
                                      +[](WalkTasks& tasks) -> PositionTask& { return *tasks.trunk_task; },
                                      return_value_policy<reference_existing_object>()));

  class__<LIPM::Trajectory>("LIPMTrajectory", init<>())
      .def("pos", &LIPM::Trajectory::pos)
      .def("vel", &LIPM::Trajectory::vel)
      .def("acc", &LIPM::Trajectory::acc)
      .def("jerk", &LIPM::Trajectory::jerk)
      .def("zmp", &LIPM::Trajectory::zmp)
      .def("dzmp", &LIPM::Trajectory::dzmp)
      .def("dcm", &LIPM::Trajectory::dcm);

  class__<LIPM>("LIPM",
                init<problem::Problem&, double, int, double, Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d>())
      .def("compute_omega", &LIPM::compute_omega)
      .def("get_trajectory", &LIPM::get_trajectory)
      .def("pos", &LIPM::pos)
      .def("vel", &LIPM::vel)
      .def("acc", &LIPM::acc)
      .def("jerk", &LIPM::jerk)
      .def("dcm", &LIPM::dcm)
      .def("zmp", &LIPM::zmp)
      .def("dzmp", &LIPM::dzmp)
      .def("build_LIPM_from_previous", &LIPM::build_LIPM_from_previous)
      .def("get_trajectory", &LIPM::get_trajectory)
      .add_property("dt", &LIPM::dt, &LIPM::dt)
      .add_property("timesteps", &LIPM::timesteps, &LIPM::timesteps)
      .add_property("t_start", &LIPM::t_start, &LIPM::t_start)
      .add_property("t_end", &LIPM::t_end, &LIPM::t_end)
      .add_property("x_var", &LIPM::x_var, &LIPM::x_var)
      .add_property("y_var", &LIPM::y_var, &LIPM::y_var)
      .add_property("x", &LIPM::x, &LIPM::x)
      .add_property("y", &LIPM::y, &LIPM::y);
}