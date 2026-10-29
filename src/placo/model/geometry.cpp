#include "placo/model/geometry.h"
#include "placo/model/data.h"
#include <map>
#include <stdexcept>
#include <coal/collision.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/mesh_loader/loader.h>
#include <coal/BVH/BVH_model.h>

namespace placo::model
{
std::string GeometryObject::type_name() const
{
  switch (type)
  {
    case BOX:
      return "box";
    case SPHERE:
      return "sphere";
    case CYLINDER:
      return "cylinder";
    case CAPSULE:
      return "capsule";
    case MESH:
      return "mesh";
    case ELLIPSOID:
      return "ellipsoid";
    case PLANE:
      return "plane";
  }
  return "unknown";
}

bool GeometryObject::operator==(const GeometryObject& other) const
{
  return name == other.name && parent_frame == other.parent_frame;
}

int GeometryModel::add_object(const GeometryObject& object)
{
  objects.push_back(object);
  return objects.size() - 1;
}

void GeometryModel::add_all_collision_pairs()
{
  remove_all_collision_pairs();
  for (int i = 0; i < (int)objects.size(); i++)
  {
    for (int j = i + 1; j < (int)objects.size(); j++)
    {
      if (objects[i].parent_joint != objects[j].parent_joint)
      {
        add_collision_pair(i, j);
      }
    }
  }
}

void GeometryModel::remove_all_collision_pairs()
{
  collision_pairs.clear();
}

void GeometryModel::add_collision_pair(int a, int b)
{
  if (a == b || a < 0 || b < 0 || a >= (int)objects.size() || b >= (int)objects.size())
  {
    throw std::runtime_error("GeometryModel::add_collision_pair: invalid pair (" + std::to_string(a) + ", " +
                             std::to_string(b) + ")");
  }

  for (auto& pair : collision_pairs)
  {
    if ((pair.first == a && pair.second == b) || (pair.first == b && pair.second == a))
    {
      return;
    }
  }
  collision_pairs.push_back(std::make_pair(a, b));
}

void GeometryModel::build_collision_geometries()
{
  // Meshes are loaded once, and shared by the geometries using the same file and scale
  coal::MeshLoader loader;
  std::map<std::pair<std::string, std::vector<double>>, std::shared_ptr<coal::CollisionGeometry>> meshes;

  for (auto& object : objects)
  {
    switch (object.type)
    {
      case GeometryObject::BOX:
        object.geometry = std::make_shared<coal::Box>(object.size.x(), object.size.y(), object.size.z());
        break;
      case GeometryObject::SPHERE:
        object.geometry = std::make_shared<coal::Sphere>(object.size.x());
        break;
      case GeometryObject::CYLINDER:
        object.geometry = std::make_shared<coal::Cylinder>(object.size.x(), object.size.y());
        break;
      case GeometryObject::CAPSULE:
        object.geometry = std::make_shared<coal::Capsule>(object.size.x(), object.size.y());
        break;
      case GeometryObject::ELLIPSOID:
        object.geometry = std::make_shared<coal::Ellipsoid>(object.size.x(), object.size.y(), object.size.z());
        break;
      case GeometryObject::PLANE:
        object.geometry = std::make_shared<coal::Halfspace>(0., 0., 1., 0.);
        break;
      case GeometryObject::MESH: {
        auto key = std::make_pair(object.mesh_path,
                                  std::vector<double>(object.mesh_scale.data(), object.mesh_scale.data() + 3));
        if (!meshes.count(key))
        {
          coal::Vec3s scale(object.mesh_scale.x(), object.mesh_scale.y(), object.mesh_scale.z());
          meshes[key] = loader.load(object.mesh_path, scale);
        }
        object.geometry = meshes[key];
        break;
      }
    }
  }
}

std::vector<Eigen::Affine3d> GeometryModel::placements(const Data& data) const
{
  std::vector<Eigen::Affine3d> result;
  result.reserve(objects.size());
  for (auto& object : objects)
  {
    result.push_back(data.oMi[object.parent_joint] * object.placement);
  }
  return result;
}

static coal::Transform3s to_coal(const Eigen::Affine3d& T)
{
  return coal::Transform3s(T.linear(), T.translation());
}

std::vector<CollisionResult> compute_collisions(const GeometryModel& geometry_model, const Data& data,
                                                bool stop_at_first)
{
  std::vector<CollisionResult> collisions;
  std::vector<Eigen::Affine3d> placements = geometry_model.placements(data);

  for (int k = 0; k < (int)geometry_model.collision_pairs.size(); k++)
  {
    const auto& pair = geometry_model.collision_pairs[k];
    const GeometryObject& a = geometry_model.objects[pair.first];
    const GeometryObject& b = geometry_model.objects[pair.second];
    if (!a.geometry || !b.geometry)
    {
      throw std::runtime_error("compute_collisions: the collision geometries are not built");
    }

    coal::CollisionRequest request(coal::NO_REQUEST, 1);
    request.enable_cached_gjk_guess = true;
    request.enable_contact = true;
    request.distance_upper_bound = request.security_margin + 1e-6;

    coal::CollisionResult result;
    coal::ComputeCollision collide(a.geometry.get(), b.geometry.get());
    collide(to_coal(placements[pair.first]), to_coal(placements[pair.second]), request, result);

    if (result.isCollision())
    {
      CollisionResult collision;
      collision.pair = k;
      for (size_t c = 0; c < result.numContacts(); c++)
      {
        collision.contacts.push_back(Eigen::Vector3d(result.getContact(c).pos));
      }
      collisions.push_back(collision);

      if (stop_at_first)
      {
        break;
      }
    }
  }

  return collisions;
}

std::vector<DistanceResult> compute_distances(const GeometryModel& geometry_model, const Data& data)
{
  std::vector<DistanceResult> distances;
  std::vector<Eigen::Affine3d> placements = geometry_model.placements(data);

  for (int k = 0; k < (int)geometry_model.collision_pairs.size(); k++)
  {
    const auto& pair = geometry_model.collision_pairs[k];
    const GeometryObject& a = geometry_model.objects[pair.first];
    const GeometryObject& b = geometry_model.objects[pair.second];
    if (!a.geometry || !b.geometry)
    {
      throw std::runtime_error("compute_distances: the collision geometries are not built");
    }

    coal::DistanceRequest request(true);
    request.enable_cached_gjk_guess = true;

    coal::DistanceResult result;
    coal::ComputeDistance distance(a.geometry.get(), b.geometry.get());
    distance(to_coal(placements[pair.first]), to_coal(placements[pair.second]), request, result);

    DistanceResult entry;
    entry.pair = k;
    entry.min_distance = result.min_distance;
    entry.point_a = result.nearest_points[0];
    entry.point_b = result.nearest_points[1];
    distances.push_back(entry);
  }

  return distances;
}
}  // namespace placo::model
