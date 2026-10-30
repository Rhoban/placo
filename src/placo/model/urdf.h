#pragma once

// PlaCo's rigid body model is a lighter, standalone implementation that uses pinocchio
// (https://github.com/stack-of-tasks/pinocchio) as its reference: it follows its conventions and formulas, so that
// the results are the same up to rounding errors.

#include <string>
#include <vector>
#include "placo/model/model.h"
#include "placo/model/geometry.h"

namespace placo::model
{
/**
 * @brief Loads a robot from URDF contents. The robot is attached to the world with a free-flyer joint named
 * "root_joint". Fixed joints are not joints of the model: their child bodies are merged with their parent (and
 * their frames are kept).
 * @param xml URDF contents
 * @param model (output) the model
 * @param collision_model (output) the collision geometries (the collision pairs are not added)
 * @param visual_model (output) the visual geometries
 * @param package_directories directories where the meshes are searched
 * @param collision_as_visual if true, the visual geometries are the collision ones
 */
void load_urdf(const std::string& xml, Model& model, GeometryModel& collision_model, GeometryModel& visual_model,
               const std::vector<std::string>& package_directories, bool collision_as_visual = false);

/**
 * @brief Absolute path of a resource (e.g. a mesh). Paths of the form "package://path" or relative paths are searched
 * in the given directories, "file://path" is the given path
 * @return the path, or an empty string if it can't be found
 */
std::string retrieve_resource_path(const std::string& path, const std::vector<std::string>& directories);
}  // namespace placo::model
