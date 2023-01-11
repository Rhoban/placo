#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/planning/walk_pattern_generator.h"
#include "placo/planning/swing_foot.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeWalkPatternGenerator()
{
  class_<WalkPatternGenerator::Trajectory>("WalkTrajectory")
      .add_property("footsteps", &WalkPatternGenerator::Trajectory::footsteps)
      .add_property("com", &WalkPatternGenerator::Trajectory::com)
      .add_property("duration", &WalkPatternGenerator::Trajectory::duration)
      .add_property("jerk_planner_steps", &WalkPatternGenerator::Trajectory::jerk_planner_steps)
      .def("get_T_world_left", &WalkPatternGenerator::Trajectory::get_T_world_left)
      .def("get_T_world_right", &WalkPatternGenerator::Trajectory::get_T_world_right)
      .def("get_CoM_world", &WalkPatternGenerator::Trajectory::get_CoM_world)
      .def("get_R_world_trunk", &WalkPatternGenerator::Trajectory::get_R_world_trunk)
      .def("support_side", &WalkPatternGenerator::Trajectory::support_side);

  class_<WalkPatternGenerator>("WalkPatternGenerator", init<HumanoidRobot&>())
      .add_property("parameters", &WalkPatternGenerator::parameters, &WalkPatternGenerator::parameters)
      .def("plan", &WalkPatternGenerator::plan_by_frames);

  class_<SwingFoot>("SwingFoot", init<>()).def("make_trajectory", &SwingFoot::make_trajectory);

  class_<SwingFoot::Trajectory>("SwingFootTrajectory", init<>())
      .def("pos", &SwingFoot::Trajectory::pos)
      .def("vel", &SwingFoot::Trajectory::vel);
}