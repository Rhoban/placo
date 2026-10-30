#pragma once

// PlaCo's rigid body model is a lighter, standalone implementation that uses pinocchio
// (https://github.com/stack-of-tasks/pinocchio) as its reference: it follows its conventions and formulas, so that
// the results are the same up to rounding errors.

#include <string>
#include "placo/model/model.h"
#include "placo/model/geometry.h"

namespace placo::model
{
/**
 * @brief Loads a robot from a MuJoCo (MJCF) file. The robot is attached to the world with a free-flyer joint named
 * "root_joint".
 * @param filename path to the MJCF file
 * @param model (output) the model
 * @param collision_model (output) the collision geometries (the collision pairs are not added)
 * @param visual_model (output) the visual geometries
 * @param collision_as_visual if true, the visual geometries are the collision ones
 */
void load_mjcf(const std::string& filename, Model& model, GeometryModel& collision_model, GeometryModel& visual_model,
               bool collision_as_visual = false);
}  // namespace placo::model
