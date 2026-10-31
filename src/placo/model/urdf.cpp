#include "placo/model/urdf.h"
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tinyxml2.h>

namespace placo::model
{
// Parses whitespace separated numbers (independently of the locale)
static std::vector<double> parse_numbers(const char* text)
{
  std::vector<double> values;
  if (text == nullptr)
  {
    return values;
  }

  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  double value;
  while (stream >> value)
  {
    values.push_back(value);
  }
  if (!stream.eof())
  {
    throw std::runtime_error(std::string("URDF: can't parse numbers in \"") + text + "\"");
  }
  return values;
}

static Eigen::Vector3d parse_vector3(const tinyxml2::XMLElement* element, const char* attribute,
                                     const Eigen::Vector3d& default_value)
{
  const char* text = element ? element->Attribute(attribute) : nullptr;
  if (text == nullptr)
  {
    return default_value;
  }
  std::vector<double> values = parse_numbers(text);
  if (values.size() != 3)
  {
    throw std::runtime_error(std::string("URDF: expected 3 values for ") + attribute + ", got \"" + text + "\"");
  }
  return Eigen::Vector3d(values[0], values[1], values[2]);
}

static double parse_double(const tinyxml2::XMLElement* element, const char* attribute, double default_value)
{
  const char* text = element ? element->Attribute(attribute) : nullptr;
  if (text == nullptr)
  {
    return default_value;
  }
  std::vector<double> values = parse_numbers(text);
  if (values.size() != 1)
  {
    throw std::runtime_error(std::string("URDF: expected a value for ") + attribute + ", got \"" + text + "\"");
  }
  return values[0];
}

// Placement given by an <origin> element (xyz and rpy), the rotation being computed as urdfdom does (through a
// quaternion)
static Eigen::Affine3d parse_origin(const tinyxml2::XMLElement* parent)
{
  const tinyxml2::XMLElement* origin = parent ? parent->FirstChildElement("origin") : nullptr;
  Eigen::Vector3d xyz = parse_vector3(origin, "xyz", Eigen::Vector3d::Zero());
  Eigen::Vector3d rpy = parse_vector3(origin, "rpy", Eigen::Vector3d::Zero());

  const double phi = rpy[0] / 2., the = rpy[1] / 2., psi = rpy[2] / 2.;
  double x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
  double y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
  double z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
  double w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
  const double norm = sqrt(x * x + y * y + z * z + w * w);
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
  T.translation() = xyz;
  return T;
}

static Inertia parse_inertia(const tinyxml2::XMLElement* link)
{
  const tinyxml2::XMLElement* inertial = link->FirstChildElement("inertial");
  if (inertial == nullptr)
  {
    return Inertia();
  }

  Eigen::Affine3d origin = parse_origin(inertial);
  double mass = parse_double(inertial->FirstChildElement("mass"), "value", 0.);

  const tinyxml2::XMLElement* inertia_element = inertial->FirstChildElement("inertia");
  Eigen::Matrix3d I;
  double ixx = parse_double(inertia_element, "ixx", 0.), ixy = parse_double(inertia_element, "ixy", 0.),
         ixz = parse_double(inertia_element, "ixz", 0.), iyy = parse_double(inertia_element, "iyy", 0.),
         iyz = parse_double(inertia_element, "iyz", 0.), izz = parse_double(inertia_element, "izz", 0.);
  I << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;

  const Eigen::Matrix3d& R = origin.linear();
  return Inertia(mass, origin.translation(), R * I * R.transpose());
}

std::string retrieve_resource_path(const std::string& path, const std::vector<std::string>& directories)
{
  namespace fs = std::filesystem;
  const std::string separator = "://";
  size_t pos = path.find(separator);

  if (pos != std::string::npos)
  {
    std::string scheme = path.substr(0, pos);
    std::string relative = path.substr(pos + separator.size());

    if (scheme == "package" || scheme == "model")
    {
      for (auto& directory : directories)
      {
        if (fs::exists(directory + "/" + relative))
        {
          return directory + "/" + relative;
        }
      }
      return "";
    }
    else if (scheme == "file")
    {
      return relative;
    }
    throw std::runtime_error("URDF: schemes of form '" + scheme + "' are not handled");
  }
  else if (fs::path(path).is_relative())
  {
    for (auto& directory : directories)
    {
      if (fs::exists(directory + "/" + path))
      {
        return directory + "/" + path;
      }
    }
    return "";
  }

  return path;
}

namespace
{
struct URDFJoint
{
  const tinyxml2::XMLElement* element;
  std::string name;
  std::string type;
  std::string parent;
  std::string child;
};

class URDFLoader
{
public:
  URDFLoader(const tinyxml2::XMLElement* robot, Model& model, GeometryModel& collision_model,
             GeometryModel& visual_model, const std::vector<std::string>& package_directories, bool collision_as_visual)
    : robot(robot)
    , model(model)
    , collision_model(collision_model)
    , visual_model(visual_model)
    , package_directories(package_directories)
    , collision_as_visual(collision_as_visual)
  {
  }

  void load()
  {
    const char* robot_name = robot->Attribute("name");
    model.name = robot_name ? robot_name : "";

    // Global materials
    for (auto* material = robot->FirstChildElement("material"); material; material = material->NextSiblingElement("mate"
                                                                                                                  "ria"
                                                                                                                  "l"))
    {
      Eigen::Vector4d color;
      const char* name = material->Attribute("name");
      if (name && parse_color(material, color))
      {
        materials[name] = color;
      }
    }

    // Links
    for (auto* link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
    {
      const char* name = link->Attribute("name");
      if (name == nullptr)
      {
        throw std::runtime_error("URDF: a link has no name");
      }
      if (links.count(name))
      {
        throw std::runtime_error(std::string("URDF: duplicate link ") + name);
      }
      links[name] = link;
    }

    // Joints, sorted by name (as urdfdom does, which gives the order of the children of each link)
    std::map<std::string, URDFJoint> joints;
    for (auto* element = robot->FirstChildElement("joint"); element; element = element->NextSiblingElement("joint"))
    {
      URDFJoint joint;
      joint.element = element;
      const char* name = element->Attribute("name");
      const char* type = element->Attribute("type");
      const tinyxml2::XMLElement* parent = element->FirstChildElement("parent");
      const tinyxml2::XMLElement* child = element->FirstChildElement("child");
      if (!name || !type || !parent || !child || !parent->Attribute("link") || !child->Attribute("link"))
      {
        throw std::runtime_error("URDF: malformed joint " + std::string(name ? name : ""));
      }
      joint.name = name;
      joint.type = type;
      joint.parent = parent->Attribute("link");
      joint.child = child->Attribute("link");
      if (!links.count(joint.parent) || !links.count(joint.child))
      {
        throw std::runtime_error("URDF: joint " + joint.name + " refers to a missing link");
      }
      if (joints.count(joint.name))
      {
        throw std::runtime_error("URDF: duplicate joint " + joint.name);
      }
      joints[joint.name] = joint;
    }

    for (auto& entry : joints)
    {
      const URDFJoint& joint = entry.second;
      if (parent_joint.count(joint.child))
      {
        throw std::runtime_error("URDF: link " + joint.child + " has several parents");
      }
      parent_joint[joint.child] = joint;
      children[joint.parent].push_back(joint.child);
    }

    // Root link
    std::string root;
    for (auto& entry : links)
    {
      if (!parent_joint.count(entry.first))
      {
        if (root != "")
        {
          throw std::runtime_error("URDF: several root links (" + root + ", " + entry.first + ")");
        }
        root = entry.first;
      }
    }
    if (root == "")
    {
      throw std::runtime_error("URDF: no root link");
    }

    // Root joint (free flyer) and root body
    int root_joint = model.add_joint(0, Joint(Joint::FREE_FLYER), Eigen::Affine3d::Identity(), "root_joint");
    int root_joint_frame = model.add_joint_frame(root_joint, 0);
    append_body(root_joint_frame, parse_inertia(links[root]), root);

    for (auto& child : children[root])
    {
      parse_tree(child);
    }

    // Geometries, in the order of the tree
    parse_geometries(root);
  }

protected:
  const tinyxml2::XMLElement* robot;
  Model& model;
  GeometryModel& collision_model;
  GeometryModel& visual_model;
  const std::vector<std::string>& package_directories;
  bool collision_as_visual;

  std::map<std::string, const tinyxml2::XMLElement*> links;
  std::map<std::string, URDFJoint> parent_joint;
  std::map<std::string, std::vector<std::string>> children;
  std::map<std::string, Eigen::Vector4d> materials;

  bool parse_color(const tinyxml2::XMLElement* material, Eigen::Vector4d& color)
  {
    const tinyxml2::XMLElement* color_element = material->FirstChildElement("color");
    if (color_element && color_element->Attribute("rgba"))
    {
      std::vector<double> rgba = parse_numbers(color_element->Attribute("rgba"));
      if (rgba.size() == 4)
      {
        color = Eigen::Vector4d(rgba[0], rgba[1], rgba[2], rgba[3]);
        return true;
      }
    }
    return false;
  }

  // Adds the frame of a body (and its inertia) attached to the frame of its joint
  void append_body(int joint_frame, const Inertia& inertia, const std::string& name)
  {
    const Frame& frame = model.frames[joint_frame];
    const Eigen::Affine3d placement = frame.placement;
    const int joint = frame.parent_joint;
    if (!inertia.is_zero())
    {
      model.append_body_to_joint(joint, inertia, placement);
    }
    model.add_body_frame(name, joint, placement, joint_frame);
  }

  int body_frame(const std::string& link)
  {
    int frame = model.get_frame_id(link, Frame::BODY);
    if (frame >= (int)model.frames.size())
    {
      throw std::runtime_error("URDF: no body frame for link " + link);
    }
    return frame;
  }

  void parse_tree(const std::string& link)
  {
    const URDFJoint& joint = parent_joint[link];
    const int parent_frame_id = body_frame(joint.parent);
    const Frame parent_frame = model.frames[parent_frame_id];

    const Eigen::Affine3d joint_placement = parse_origin(joint.element);
    const Inertia inertia = parse_inertia(links[link]);
    const double infinity = std::numeric_limits<double>::infinity();

    if (joint.type == "fixed")
    {
      // The body is merged with the parent one, the frames are kept
      Frame frame;
      frame.name = joint.name;
      frame.parent_joint = parent_frame.parent_joint;
      frame.parent_frame = parent_frame_id;
      frame.placement = parent_frame.placement * joint_placement;
      frame.type = Frame::FIXED_JOINT;
      frame.inertia = inertia;
      int fixed_frame = model.add_frame(frame);
      model.add_body_frame(link, frame.parent_joint, frame.placement, fixed_frame);
    }
    else
    {
      const tinyxml2::XMLElement* axis_element = joint.element->FirstChildElement("axis");
      Eigen::Vector3d axis = parse_vector3(axis_element, "xyz", Eigen::Vector3d::UnitX());
      const tinyxml2::XMLElement* limit = joint.element->FirstChildElement("limit");

      Joint model_joint;
      Eigen::VectorXd max_effort, max_velocity, min_config, max_config;

      if (joint.type == "revolute" || joint.type == "prismatic")
      {
        if (limit == nullptr)
        {
          throw std::runtime_error("URDF: joint " + joint.name + " of type " + joint.type + " has no limits");
        }
        model_joint = Joint(joint.type == "revolute" ? Joint::REVOLUTE : Joint::PRISMATIC, axis);
        max_effort = Eigen::VectorXd::Constant(1, parse_double(limit, "effort", 0.));
        max_velocity = Eigen::VectorXd::Constant(1, parse_double(limit, "velocity", 0.));
        min_config = Eigen::VectorXd::Constant(1, parse_double(limit, "lower", 0.));
        max_config = Eigen::VectorXd::Constant(1, parse_double(limit, "upper", 0.));
      }
      else if (joint.type == "continuous")
      {
        model_joint = Joint(Joint::REVOLUTE_UNBOUNDED, axis);
        max_effort = Eigen::VectorXd::Constant(1, limit ? parse_double(limit, "effort", 0.) : infinity);
        max_velocity = Eigen::VectorXd::Constant(1, limit ? parse_double(limit, "velocity", 0.) : infinity);
        min_config = Eigen::VectorXd::Constant(2, -1.01);
        max_config = Eigen::VectorXd::Constant(2, 1.01);
      }
      else if (joint.type == "floating")
      {
        model_joint = Joint(Joint::FREE_FLYER);
        max_effort = Eigen::VectorXd::Constant(6, infinity);
        max_velocity = Eigen::VectorXd::Constant(6, infinity);
        min_config = Eigen::VectorXd::Constant(7, -infinity);
        max_config = Eigen::VectorXd::Constant(7, infinity);
        min_config.tail(4).setConstant(-1.01);
        max_config.tail(4).setConstant(1.01);
      }
      else
      {
        throw std::runtime_error("URDF: the type " + joint.type + " of joint " + joint.name + " is not supported");
      }

      int joint_id = model.add_joint(parent_frame.parent_joint, model_joint, parent_frame.placement * joint_placement,
                                     joint.name, max_effort, max_velocity, min_config, max_config);
      int joint_frame = model.add_joint_frame(joint_id, parent_frame_id);
      append_body(joint_frame, inertia, link);
    }

    for (auto& child : children[link])
    {
      parse_tree(child);
    }
  }

  bool is_capsule(const tinyxml2::XMLElement* link, const char* geometry_name)
  {
    if (geometry_name == nullptr)
    {
      return false;
    }
    for (auto* checking = link->FirstChildElement("collision_checking"); checking;
         checking = checking->NextSiblingElement("collision_checking"))
    {
      for (auto* capsule = checking->FirstChildElement("capsule"); capsule; capsule = capsule->NextSiblingElement("caps"
                                                                                                                  "ul"
                                                                                                                  "e"))
      {
        if (capsule->Attribute("name") && std::string(capsule->Attribute("name")) == geometry_name)
        {
          return true;
        }
      }
    }
    return false;
  }

  void add_geometries(const std::string& link_name, const char* tag, GeometryModel& geometry_model, bool visual)
  {
    const tinyxml2::XMLElement* link = links[link_name];
    const int frame_id = body_frame(link_name);
    const Frame& frame = model.frames[frame_id];

    int index = 0;
    for (auto* element = link->FirstChildElement(tag); element; element = element->NextSiblingElement(tag))
    {
      GeometryObject object;
      object.name = link_name + "_" + std::to_string(index++);
      object.parent_joint = frame.parent_joint;
      object.parent_frame = frame_id;
      object.placement = frame.placement * parse_origin(element);

      const tinyxml2::XMLElement* geometry = element->FirstChildElement("geometry");
      const tinyxml2::XMLElement* shape = geometry ? geometry->FirstChildElement() : nullptr;
      if (shape == nullptr)
      {
        throw std::runtime_error("URDF: a geometry of link " + link_name + " has no shape");
      }

      std::string shape_type = shape->Name();
      if (shape_type == "box")
      {
        object.type = GeometryObject::BOX;
        object.size = parse_vector3(shape, "size", Eigen::Vector3d::Zero());
      }
      else if (shape_type == "sphere")
      {
        object.type = GeometryObject::SPHERE;
        object.size = Eigen::Vector3d(parse_double(shape, "radius", 0.), 0., 0.);
      }
      else if (shape_type == "cylinder" || shape_type == "capsule")
      {
        object.type = (shape_type == "capsule" || is_capsule(link, element->Attribute("name"))) ?
                          GeometryObject::CAPSULE :
                          GeometryObject::CYLINDER;
        object.size = Eigen::Vector3d(parse_double(shape, "radius", 0.), parse_double(shape, "length", 0.), 0.);
      }
      else if (shape_type == "mesh")
      {
        object.type = GeometryObject::MESH;
        const char* filename = shape->Attribute("filename");
        if (filename == nullptr)
        {
          throw std::runtime_error("URDF: a mesh of link " + link_name + " has no filename");
        }
        object.mesh_path = retrieve_resource_path(filename, package_directories);
        if (object.mesh_path == "")
        {
          throw std::runtime_error(std::string("Mesh ") + filename + " could not be found.");
        }
        object.mesh_scale = parse_vector3(shape, "scale", Eigen::Vector3d::Ones());
      }
      else
      {
        throw std::runtime_error("URDF: unknown geometry type " + shape_type);
      }

      if (visual)
      {
        const tinyxml2::XMLElement* material = element->FirstChildElement("material");
        if (material)
        {
          object.has_color = true;
          object.color = Eigen::Vector4d::Zero();
          if (!parse_color(material, object.color) && material->Attribute("name") &&
              materials.count(material->Attribute("name")))
          {
            object.color = materials[material->Attribute("name")];
          }
        }
      }

      geometry_model.add_object(object);
    }
  }

  void parse_geometries(const std::string& link)
  {
    add_geometries(link, "collision", collision_model, false);
    if (collision_as_visual)
    {
      add_geometries(link, "collision", visual_model, false);
    }
    else
    {
      add_geometries(link, "visual", visual_model, true);
    }

    for (auto& child : children[link])
    {
      parse_geometries(child);
    }
  }
};
}  // namespace

void load_urdf(const std::string& xml, Model& model, GeometryModel& collision_model, GeometryModel& visual_model,
               const std::vector<std::string>& package_directories, bool collision_as_visual)
{
  tinyxml2::XMLDocument document;
  if (document.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
  {
    throw std::runtime_error(std::string("URDF: can't parse the XML: ") + document.ErrorStr());
  }

  const tinyxml2::XMLElement* robot = document.FirstChildElement("robot");
  if (robot == nullptr)
  {
    throw std::runtime_error("URDF: no <robot> element");
  }

  // Meshes are also searched in the ROS package path
  std::vector<std::string> directories = package_directories;
  const char* ros_package_path = std::getenv("ROS_PACKAGE_PATH");
  if (ros_package_path)
  {
    std::istringstream stream(ros_package_path);
    std::string directory;
    while (std::getline(stream, directory, ':'))
    {
      if (directory != "")
      {
        directories.push_back(directory);
      }
    }
  }

  URDFLoader loader(robot, model, collision_model, visual_model, directories, collision_as_visual);
  loader.load();
}
}  // namespace placo::model
