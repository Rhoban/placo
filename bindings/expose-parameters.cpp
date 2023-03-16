#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "placo/model/robot_wrapper.h"
#include "placo/model/humanoid_robot.h"
#include "placo/model/humanoid_parameters.h"
#include <Eigen/Dense>
#include <boost/python.hpp>

using namespace boost::python;
using namespace placo;

void exposeParameters()
{
  class_<HumanoidParameters>("HumanoidParameters")
      .add_property("dt", &HumanoidParameters::dt, &HumanoidParameters::dt)
      .add_property("single_support_duration", &HumanoidParameters::single_support_duration,
                    &HumanoidParameters::single_support_duration)
      .add_property("double_support_duration", &HumanoidParameters::double_support_duration,
                    &HumanoidParameters::double_support_duration)
      .add_property("startend_double_support_duration", &HumanoidParameters::startend_double_support_duration,
                    &HumanoidParameters::startend_double_support_duration)
      .add_property("kick_duration", &HumanoidParameters::kick_duration, &HumanoidParameters::kick_duration)
      .add_property("planned_dt", &HumanoidParameters::planned_dt, &HumanoidParameters::planned_dt)
      .add_property("replan_frequency", &HumanoidParameters::replan_frequency, &HumanoidParameters::replan_frequency)
      .add_property("zmp_margin", &HumanoidParameters::zmp_margin, &HumanoidParameters::zmp_margin)
      .add_property("walk_foot_height", &HumanoidParameters::walk_foot_height, &HumanoidParameters::walk_foot_height)
      .add_property("walk_com_height", &HumanoidParameters::walk_com_height, &HumanoidParameters::walk_com_height)
      .add_property("walk_trunk_pitch", &HumanoidParameters::walk_trunk_pitch, &HumanoidParameters::walk_trunk_pitch)
      .add_property("walk_foot_tilt", &HumanoidParameters::walk_foot_tilt, &HumanoidParameters::walk_foot_tilt)
      .add_property("pendulum_height", &HumanoidParameters::pendulum_height, &HumanoidParameters::pendulum_height)
      .add_property("feet_spacing", &HumanoidParameters::feet_spacing, &HumanoidParameters::feet_spacing)
      .add_property("foot_width", &HumanoidParameters::foot_width, &HumanoidParameters::foot_width)
      .add_property("foot_length", &HumanoidParameters::foot_length, &HumanoidParameters::foot_length)

      ;
}