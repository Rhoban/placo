#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/planning/walk_pattern_generator.h"
#include "placo/control/kinematics_solver.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/planning/swing_foot_quintic.h"
#include "placo/planning/walk_tasks.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeWalkPatternGenerator()
{
  class_<WalkPatternGenerator::Trajectory>("WalkTrajectory")
      .add_property("supports", &WalkPatternGenerator::Trajectory::supports)
      .add_property("com", &WalkPatternGenerator::Trajectory::com)
      .add_property("t_start", &WalkPatternGenerator::Trajectory::t_start)
      .add_property("t_end", &WalkPatternGenerator::Trajectory::t_end)
      .add_property("jerk_planner_nb_dt", &WalkPatternGenerator::Trajectory::jerk_planner_nb_dt)
      .def("get_T_world_left", &WalkPatternGenerator::Trajectory::get_T_world_left)
      .def("get_T_world_right", &WalkPatternGenerator::Trajectory::get_T_world_right)
      .def("get_p_world_CoM", &WalkPatternGenerator::Trajectory::get_p_world_CoM)
      .def("get_R_world_trunk", &WalkPatternGenerator::Trajectory::get_R_world_trunk)
      .def("support_side", &WalkPatternGenerator::Trajectory::support_side)
      .def("is_both_support", &WalkPatternGenerator::Trajectory::is_both_support)
      .def("get_support", &WalkPatternGenerator::Trajectory::get_support)
      .def("get_next_support", &WalkPatternGenerator::Trajectory::get_next_support)
      .def("get_prev_support", &WalkPatternGenerator::Trajectory::get_prev_support)
      .def("get_part_t_start", &WalkPatternGenerator::Trajectory::get_part_t_start);

  class_<WalkPatternGenerator>("WalkPatternGenerator", init<HumanoidRobot&, HumanoidParameters&>())
      .def("plan", &WalkPatternGenerator::plan)
      .def("replan", &WalkPatternGenerator::replan)
      .def("replan_supports", &WalkPatternGenerator::replan_supports);

  class_<SwingFoot>("SwingFoot", init<>())
      .def("make_trajectory", &SwingFoot::make_trajectory)
      .def("remake_trajectory", &SwingFoot::remake_trajectory);

  class_<SwingFoot::Trajectory>("SwingFootTrajectory", init<>())
      .def("pos", &SwingFoot::Trajectory::pos)
      .def("vel", &SwingFoot::Trajectory::vel);

  class_<SwingFootQuintic>("SwingFootQuintic", init<>()).def("make_trajectory", &SwingFootQuintic::make_trajectory);

  class_<SwingFootQuintic::Trajectory>("SwingFootQuinticTrajectory", init<>())
      .def("pos", &SwingFootQuintic::Trajectory::pos)
      .def("vel", &SwingFootQuintic::Trajectory::vel);

  class_<WalkTasks>("WalkTasks", init<>())
      .def(
          "initialize_tasks", +[](WalkTasks& tasks, KinematicsSolver& solver) { tasks.initialize_tasks(&solver); })
      .def("update_tasks", &WalkTasks::update_tasks)
      .def("remove_tasks", &WalkTasks::remove_tasks)
      .add_property(
          "solver", +[](WalkTasks& tasks) { return *tasks.solver; })
      .add_property("left_foot_task", &WalkTasks::left_foot_task)
      .add_property("right_foot_task", &WalkTasks::right_foot_task)
      .add_property(
          "com_task", +[](WalkTasks& tasks) { return *tasks.com_task; })
      .add_property(
          "trunk_orientation_task", +[](WalkTasks& tasks) { return *tasks.trunk_orientation_task; });
}