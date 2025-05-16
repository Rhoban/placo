#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "doxystub.h"
#include "placo/model/robot_wrapper.h"
#include "placo/humanoid/humanoid_robot.h"
#include "placo/humanoid/humanoid_parameters.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigen-to-python.hpp>

using namespace boost::python;
using namespace placo;
using namespace placo::model;
using namespace placo::humanoid;

void exposeParameters()
{
  class__<HumanoidParameters>("HumanoidParameters")
      .add_property("single_support_duration", &HumanoidParameters::single_support_duration,
                    &HumanoidParameters::single_support_duration)
      .add_property("single_support_timesteps", &HumanoidParameters::single_support_timesteps,
                    &HumanoidParameters::single_support_timesteps)
      .add_property("double_support_ratio", &HumanoidParameters::double_support_ratio,
                    &HumanoidParameters::double_support_ratio)
      .add_property("startend_double_support_ratio", &HumanoidParameters::startend_double_support_ratio,
                    &HumanoidParameters::startend_double_support_ratio)
      .add_property("planned_timesteps", &HumanoidParameters::planned_timesteps, &HumanoidParameters::planned_timesteps)
      .add_property("zmp_margin", &HumanoidParameters::zmp_margin, &HumanoidParameters::zmp_margin)
      .add_property("walk_foot_height", &HumanoidParameters::walk_foot_height, &HumanoidParameters::walk_foot_height)
      .add_property("walk_foot_rise_ratio", &HumanoidParameters::walk_foot_rise_ratio,
                    &HumanoidParameters::walk_foot_rise_ratio)
      .add_property("walk_com_height", &HumanoidParameters::walk_com_height, &HumanoidParameters::walk_com_height)
      .add_property("walk_trunk_pitch", &HumanoidParameters::walk_trunk_pitch, &HumanoidParameters::walk_trunk_pitch)
      .add_property("feet_spacing", &HumanoidParameters::feet_spacing, &HumanoidParameters::feet_spacing)
      .add_property("foot_width", &HumanoidParameters::foot_width, &HumanoidParameters::foot_width)
      .add_property("foot_length", &HumanoidParameters::foot_length, &HumanoidParameters::foot_length)
      .add_property("foot_zmp_target_x", &HumanoidParameters::foot_zmp_target_x, &HumanoidParameters::foot_zmp_target_x)
      .add_property("foot_zmp_target_y", &HumanoidParameters::foot_zmp_target_y, &HumanoidParameters::foot_zmp_target_y)
      .add_property("zmp_reference_weight", &HumanoidParameters::zmp_reference_weight,
                    &HumanoidParameters::zmp_reference_weight)
      .add_property("walk_max_dx_forward", &HumanoidParameters::walk_max_dx_forward,
                    &HumanoidParameters::walk_max_dx_forward)
      .add_property("walk_max_dx_backward", &HumanoidParameters::walk_max_dx_backward,
                    &HumanoidParameters::walk_max_dx_backward)
      .add_property("walk_max_dy", &HumanoidParameters::walk_max_dy, &HumanoidParameters::walk_max_dy)
      .add_property("walk_max_dtheta", &HumanoidParameters::walk_max_dtheta, &HumanoidParameters::walk_max_dtheta)
      .add_property("walk_dtheta_spacing", &HumanoidParameters::walk_dtheta_spacing,
                    &HumanoidParameters::walk_dtheta_spacing)
      .add_property("op_space_polygon", &HumanoidParameters::op_space_polygon, &HumanoidParameters::op_space_polygon)
      .add_property("dcm_offset_polygon", &HumanoidParameters::dcm_offset_polygon, &HumanoidParameters::dcm_offset_polygon)
      .def("dt", &HumanoidParameters::dt)
      .def("double_support_timesteps", &HumanoidParameters::double_support_timesteps)
      .def("startend_double_support_timesteps", &HumanoidParameters::startend_double_support_timesteps)
      .def("double_support_duration", &HumanoidParameters::double_support_duration)
      .def("startend_double_support_duration", &HumanoidParameters::startend_double_support_duration)
      .def("has_double_support", &HumanoidParameters::has_double_support)
      .def("ellipsoid_clip", &HumanoidParameters::ellipsoid_clip);
}