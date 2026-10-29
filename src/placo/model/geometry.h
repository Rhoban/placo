#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>

namespace coal
{
class CollisionGeometry;
}

namespace placo::model
{
class Model;
struct Data;

/**
 * @brief A geometry (collision or visual) attached to a joint
 */
struct GeometryObject
{
  enum Type
  {
    BOX = 0,
    SPHERE = 1,
    CYLINDER = 2,
    CAPSULE = 3,
    MESH = 4,
    ELLIPSOID = 5,
    PLANE = 6
  };

  /**
   * @brief Geometry name (body name, followed by the index of the geometry in the body)
   */
  std::string name;

  /**
   * @brief Joint the geometry is attached to
   */
  int parent_joint = 0;

  /**
   * @brief Frame (body) the geometry is attached to
   */
  int parent_frame = 0;

  /**
   * @brief Placement of the geometry relative to its parent joint
   */
  Eigen::Affine3d placement = Eigen::Affine3d::Identity();

  /**
   * @brief Geometry type
   */
  Type type = BOX;

  /**
   * @brief Box sizes, or (radius, length, 0) for a cylinder or capsule (along z), (radius, 0, 0) for a sphere,
   * the radii for an ellipsoid. A plane is the plane z = 0 (the geometry being z <= 0)
   */
  Eigen::Vector3d size = Eigen::Vector3d::Zero();

  /**
   * @brief Absolute path of the mesh file (meshes)
   */
  std::string mesh_path;

  /**
   * @brief Scale of the mesh
   */
  Eigen::Vector3d mesh_scale = Eigen::Vector3d::Ones();

  /**
   * @brief Color (RGBA)
   */
  Eigen::Vector4d color = Eigen::Vector4d(0.9, 0.9, 0.9, 1.);

  /**
   * @brief True if the color was specified in the model
   */
  bool has_color = false;

  /**
   * @brief Collision geometry (coal), only built for collision geometries
   */
  std::shared_ptr<coal::CollisionGeometry> geometry;

  /**
   * @brief Name of the geometry type
   */
  std::string type_name() const;

  bool operator==(const GeometryObject& other) const;
};

/**
 * @brief Geometries of a robot (collision or visual), with the pairs of geometries checked for collisions
 */
class GeometryModel
{
public:
  /**
   * @brief Geometries
   */
  std::vector<GeometryObject> objects;

  /**
   * @brief Pairs of geometries (indexes in objects) checked for collisions
   */
  std::vector<std::pair<int, int>> collision_pairs;

  /**
   * @brief Adds a geometry, returns its index
   */
  int add_object(const GeometryObject& object);

  /**
   * @brief Adds all the pairs of geometries that are not attached to the same joint
   */
  void add_all_collision_pairs();

  /**
   * @brief Removes all the collision pairs
   */
  void remove_all_collision_pairs();

  /**
   * @brief Adds a collision pair (if it is not already present)
   */
  void add_collision_pair(int a, int b);

  /**
   * @brief Builds the collision geometries (coal), loading the meshes
   */
  void build_collision_geometries();

  /**
   * @brief Placement of the geometries in the world, from the placements of the joints (data.oMi)
   */
  std::vector<Eigen::Affine3d> placements(const Data& data) const;
};

/**
 * @brief Result of a collision check between two geometries
 */
struct CollisionResult
{
  /**
   * @brief Index of the collision pair
   */
  int pair;

  /**
   * @brief Contact points (world)
   */
  std::vector<Eigen::Vector3d> contacts;
};

/**
 * @brief Checks the collisions between the pairs of geometries, for the placements of the joints (data.oMi)
 * @param geometry_model collision geometries
 * @param data data, with the joints placements
 * @param stop_at_first if true, stops at the first collision
 * @return the colliding pairs
 */
std::vector<CollisionResult> compute_collisions(const GeometryModel& geometry_model, const Data& data,
                                                bool stop_at_first);

/**
 * @brief Result of a distance computation between two geometries
 */
struct DistanceResult
{
  /**
   * @brief Index of the collision pair
   */
  int pair;

  /**
   * @brief Minimum distance between the geometries (negative if they penetrate)
   */
  double min_distance;

  /**
   * @brief Nearest points on each geometry (world)
   */
  Eigen::Vector3d point_a, point_b;
};

/**
 * @brief Computes the distances between the pairs of geometries, for the placements of the joints (data.oMi)
 */
std::vector<DistanceResult> compute_distances(const GeometryModel& geometry_model, const Data& data);
}  // namespace placo::model
