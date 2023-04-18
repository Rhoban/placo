#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/planning/walk_pattern_generator.h"
#include "placo/control/kinematics_solver.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/planning/swing_foot_quintic.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeWalkPatternGenerator()
{
  class_<WalkPatternGenerator::Trajectory>("WalkTrajectory")
      .add_property("supports", &WalkPatternGenerator::Trajectory::supports)
      .add_property("com", &WalkPatternGenerator::Trajectory::com)
      .add_property("duration", &WalkPatternGenerator::Trajectory::duration)
      .add_property("jerk_planner_nb_dt", &WalkPatternGenerator::Trajectory::jerk_planner_nb_dt)
      .add_property("time_offset", &WalkPatternGenerator::Trajectory::time_offset)
      .add_property("supports_update_offset", &WalkPatternGenerator::Trajectory::supports_update_offset)
      .add_property("are_supports_updatable", &WalkPatternGenerator::Trajectory::are_supports_updatable)
      .add_property("initial_T_world_flying_foot", &WalkPatternGenerator::Trajectory::initial_T_world_flying_foot)
      .def("get_T_world_left", &WalkPatternGenerator::Trajectory::get_T_world_left)
      .def("get_T_world_right", &WalkPatternGenerator::Trajectory::get_T_world_right)
      .def("get_p_world_CoM", &WalkPatternGenerator::Trajectory::get_p_world_CoM)
      .def("get_R_world_trunk", &WalkPatternGenerator::Trajectory::get_R_world_trunk)
      .def("support_side", &WalkPatternGenerator::Trajectory::support_side)
      .def("get_support", &WalkPatternGenerator::Trajectory::get_support)
      .def("get_next_support", &WalkPatternGenerator::Trajectory::get_next_support)
      .def("get_prev_support", &WalkPatternGenerator::Trajectory::get_prev_support)
      .def("get_phase_t_start", &WalkPatternGenerator::Trajectory::get_phase_t_start)
      .def(
          "set_supports_update_offset",
          +[](WalkPatternGenerator::Trajectory& trajectory, double t) { trajectory.supports_update_offset = t; })
      .def(
          "set_initial_T_world_flying_foot", +[](WalkPatternGenerator::Trajectory& trajectory, Eigen::Affine3d T) {
            trajectory.initial_T_world_flying_foot = T;
          });

  class_<WalkPatternGenerator>("WalkPatternGenerator", init<HumanoidRobot&, HumanoidParameters&>())
      .def("plan", &WalkPatternGenerator::plan)
      .def("replan", &WalkPatternGenerator::replan);

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
}