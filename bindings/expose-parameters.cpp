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
      .add_property("single_support_duration", &HumanoidParameters::single_support_duration,
                    &HumanoidParameters::single_support_duration)
      .add_property("single_support_timesteps", &HumanoidParameters::single_support_timesteps,
                    &HumanoidParameters::single_support_timesteps)
      .add_property("double_support_ratio", &HumanoidParameters::double_support_ratio,
                    &HumanoidParameters::double_support_ratio)
      .add_property("startend_double_support_ratio", &HumanoidParameters::startend_double_support_ratio,
                    &HumanoidParameters::startend_double_support_ratio)
      .add_property("kick_duration", &HumanoidParameters::kick_duration, &HumanoidParameters::kick_duration)
      .add_property("planned_timesteps", &HumanoidParameters::planned_timesteps, &HumanoidParameters::planned_timesteps)
      .add_property("replan_frequency", &HumanoidParameters::replan_timesteps, &HumanoidParameters::replan_timesteps)
      .add_property("zmp_margin", &HumanoidParameters::zmp_margin, &HumanoidParameters::zmp_margin)
      .add_property("walk_foot_height", &HumanoidParameters::walk_foot_height, &HumanoidParameters::walk_foot_height)
      .add_property("walk_com_height", &HumanoidParameters::walk_com_height, &HumanoidParameters::walk_com_height)
      .add_property("walk_trunk_pitch", &HumanoidParameters::walk_trunk_pitch, &HumanoidParameters::walk_trunk_pitch)
      .add_property("walk_foot_tilt", &HumanoidParameters::walk_foot_tilt, &HumanoidParameters::walk_foot_tilt)
      .add_property("pendulum_height", &HumanoidParameters::pendulum_height, &HumanoidParameters::pendulum_height)
      .add_property("feet_spacing", &HumanoidParameters::feet_spacing, &HumanoidParameters::feet_spacing)
      .add_property("foot_width", &HumanoidParameters::foot_width, &HumanoidParameters::foot_width)
      .add_property("foot_length", &HumanoidParameters::foot_length, &HumanoidParameters::foot_length)
      .def("dt", &HumanoidParameters::dt)
      .def("double_support_duration", &HumanoidParameters::double_support_duration)
      .def("startend_double_support_duration", &HumanoidParameters::startend_double_support_duration)
      .def("double_support_timesteps", &HumanoidParameters::double_support_timesteps)
      .def("startend_double_support_timesteps", &HumanoidParameters::startend_double_support_timesteps)
      .def("has_double_support", &HumanoidParameters::has_double_support)
      .def("omega", &HumanoidParameters::omega);
}