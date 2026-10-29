#include "placo/model/mjcf.h"
#include "placo/model/algorithms.h"
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <tinyxml2.h>

namespace placo::model
{
namespace
{
// Attributes of an XML element (or of the defaults of a class), by name
typedef std::map<std::string, std::string> Attributes;

// Default attributes of a class, by element type (joint, geom, site, material)
typedef std::map<std::string, Attributes> ClassDefaults;

static Attributes attributes(const tinyxml2::XMLElement* element)
{
  Attributes result;
  for (const tinyxml2::XMLAttribute* attribute = element->FirstAttribute(); attribute; attribute = attribute->Next())
  {
    result[attribute->Name()] = attribute->Value();
  }
  return result;
}

static Eigen::VectorXd parse_numbers(const std::string& text, int expected = -1)
{
  std::istringstream stream(text);
  stream.imbue(std::locale::classic());
  std::vector<double> values;
  double value;
  while (stream >> value)
  {
    values.push_back(value);
  }
  if (!stream.eof() || (expected >= 0 && (int)values.size() < expected))
  {
    throw std::runtime_error("MJCF: can't parse \"" + text + "\" (" + std::to_string(expected) + " values expected)");
  }
  Eigen::VectorXd result(expected >= 0 ? expected : values.size());
  for (int k = 0; k < result.size(); k++)
  {
    result[k] = values[k];
  }
  return result;
}

static bool has(const Attributes& attributes, const std::string& name)
{
  return attributes.count(name) > 0;
}

struct Compiler
{
  bool autolimits = true;
  bool strippath = false;
  std::string meshdir;
  double angle_converter = M_PI / 180.;
  Eigen::Matrix3d euler_axes = Eigen::Matrix3d::Identity();
  double bound_mass = 0.;
  double bound_inertia = 0.;
  // -1: auto, 0: false, 1: true
  int inertia_from_geom = -1;

  double angle(double value) const
  {
    return value * angle_converter;
  }
};

struct MJCFJoint
{
  std::string name;
  std::string type = "hinge";
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  Eigen::Affine3d placement = Eigen::Affine3d::Identity();
  double min_config = -std::numeric_limits<double>::infinity();
  double max_config = std::numeric_limits<double>::infinity();
  double max_effort = std::numeric_limits<double>::infinity();
  double armature = 0.;
  double ref = 0.;
};

struct MJCFGeom
{
  enum Kind
  {
    VISUAL,
    COLLISION,
    BOTH
  };

  std::string name;
  std::string type = "sphere";
  std::string material;
  std::string mesh;
  int contype = 1;
  int conaffinity = 1;
  int group = 0;
  std::string size_text;
  std::string fromto;
  Eigen::VectorXd size;
  Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
  double density = 1000.;
  bool has_mass = false;
  double mass = 0.;
  Eigen::Affine3d placement = Eigen::Affine3d::Identity();
  // As pinocchio, geometries whose inertia can't be computed (meshes) have a unit inertia
  Inertia inertia = Inertia(1., Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
  Kind kind = BOTH;
};

struct MJCFSite
{
  std::string name;
  Eigen::Affine3d placement = Eigen::Affine3d::Identity();
};

struct MJCFBody
{
  std::string name;
  std::string parent;
  std::string child_class;
  Eigen::Affine3d placement = Eigen::Affine3d::Identity();
  Inertia inertia;
  std::vector<MJCFJoint> joints;
  std::vector<MJCFGeom> geoms;
  std::vector<MJCFSite> sites;
};

struct MJCFMesh
{
  std::string path;
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
};

class MJCFLoader
{
public:
  MJCFLoader(const std::string& filename, Model& model, GeometryModel& collision_model, GeometryModel& visual_model,
             bool collision_as_visual)
    : filename(filename)
    , model(model)
    , collision_model(collision_model)
    , visual_model(visual_model)
    , collision_as_visual(collision_as_visual)
  {
  }

  void load()
  {
    tinyxml2::XMLDocument document;
    if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS)
    {
      throw std::runtime_error("MJCF: can't load " + filename + ": " + document.ErrorStr());
    }
    const tinyxml2::XMLElement* mujoco = document.FirstChildElement("mujoco");
    if (mujoco == nullptr)
    {
      throw std::runtime_error("MJCF: no <mujoco> element in " + filename);
    }
    if (mujoco->Attribute("model"))
    {
      model.name = mujoco->Attribute("model");
    }

    for (auto* element = mujoco->FirstChildElement(); element; element = element->NextSiblingElement())
    {
      std::string tag = element->Name();
      if (tag == "compiler")
      {
        parse_compiler(element);
      }
      else if (tag == "default")
      {
        parse_default(element, nullptr, true);
      }
      else if (tag == "asset")
      {
        parse_asset(element);
      }
      else if (tag == "worldbody")
      {
        world.name = "universe";
        for (auto* geom = element->FirstChildElement("geom"); geom; geom = geom->NextSiblingElement("geom"))
        {
          world.geoms.push_back(parse_geom(geom, world));
        }
        const tinyxml2::XMLElement* root = element->FirstChildElement("body");
        if (root)
        {
          parse_body(root, "", "");
        }
      }
    }

    if (bodies_order.empty())
    {
      throw std::runtime_error("MJCF: no body found in " + filename);
    }

    build_model();
    build_geometries(collision_model, false);
    build_geometries(visual_model, !collision_as_visual);
  }

protected:
  std::string filename;
  Model& model;
  GeometryModel& collision_model;
  GeometryModel& visual_model;
  bool collision_as_visual;

  Compiler compiler;
  std::map<std::string, ClassDefaults> classes;
  bool has_main_defaults = false;
  ClassDefaults main_defaults;
  std::map<std::string, MJCFMesh> meshes;
  std::map<std::string, Eigen::Vector4d> materials;
  std::map<std::string, MJCFBody> bodies;
  std::vector<std::string> bodies_order;
  MJCFBody world;
  Eigen::VectorXd reference_configuration;

  void parse_compiler(const tinyxml2::XMLElement* element)
  {
    Attributes a = attributes(element);
    if (has(a, "autolimits") && a["autolimits"] == "true")
    {
      compiler.autolimits = true;
    }
    if (has(a, "strippath") && a["strippath"] == "true")
    {
      compiler.strippath = true;
    }
    if (has(a, "assetdir"))
    {
      compiler.meshdir = a["assetdir"];
    }
    if (has(a, "meshdir"))
    {
      compiler.meshdir = a["meshdir"];
    }
    if (has(a, "boundmass"))
    {
      compiler.bound_mass = parse_numbers(a["boundmass"], 1)[0];
    }
    if (has(a, "boundinertia"))
    {
      compiler.bound_inertia = parse_numbers(a["boundinertia"], 1)[0];
    }
    if (has(a, "inertiafromgeom"))
    {
      if (a["inertiafromgeom"] == "true")
      {
        compiler.inertia_from_geom = 1;
      }
      else if (a["inertiafromgeom"] == "false")
      {
        compiler.inertia_from_geom = 0;
      }
    }
    if (has(a, "angle") && a["angle"] == "radian")
    {
      compiler.angle_converter = 1.;
    }
    if (has(a, "eulerseq"))
    {
      std::string sequence = a["eulerseq"];
      if (sequence.size() != 3 || sequence.find_first_not_of("xyzXYZ") != std::string::npos)
      {
        throw std::runtime_error("MJCF: invalid euler sequence " + sequence);
      }
      for (int k = 0; k < 3; k++)
      {
        char axis = std::tolower(sequence[k]);
        compiler.euler_axes.col(k) = Eigen::Vector3d::Unit(axis - 'x');
      }
    }
  }

  // Defaults: each class element gets the joint, geom and site attributes of its parent class, overridden by its
  // own; the top-level defaults (without class) apply to all the elements
  void parse_default(const tinyxml2::XMLElement* element, const ClassDefaults* parent, bool top_level)
  {
    ClassDefaults current;
    bool has_children = false;
    for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
    {
      if (std::string(child->Name()) != "default")
      {
        current[child->Name()] = attributes(child);
        has_children = true;
      }
    }

    const char* class_name = element->Attribute("class");
    if (class_name)
    {
      if (parent)
      {
        for (auto& type : { "joint", "geom", "site" })
        {
          if (parent->count(type))
          {
            Attributes merged = parent->at(type);
            for (auto& entry : current[type])
            {
              merged[entry.first] = entry.second;
            }
            current[type] = merged;
          }
        }
      }
      classes[class_name] = current;
    }
    else if (element->FirstAttribute())
    {
      throw std::runtime_error("MJCF: a default class has no name");
    }

    if (top_level && has_children && !has_main_defaults)
    {
      main_defaults = current;
      has_main_defaults = true;
    }

    for (auto* child = element->FirstChildElement("default"); child; child = child->NextSiblingElement("default"))
    {
      parse_default(child, &current, false);
    }
  }

  // Attributes of an element, from the defaults (top level, body child class, element class) and the element
  std::vector<Attributes> element_attributes(const tinyxml2::XMLElement* element, const std::string& type,
                                             const std::string& child_class)
  {
    std::vector<Attributes> result;
    if (has_main_defaults && main_defaults.count(type))
    {
      result.push_back(main_defaults[type]);
    }
    if (child_class != "")
    {
      if (!classes.count(child_class))
      {
        throw std::runtime_error("MJCF: unknown class " + child_class);
      }
      if (classes[child_class].count(type))
      {
        result.push_back(classes[child_class][type]);
      }
    }
    if (element->Attribute("class"))
    {
      std::string class_name = element->Attribute("class");
      if (!classes.count(class_name))
      {
        throw std::runtime_error("MJCF: unknown class " + class_name);
      }
      if (classes[class_name].count(type))
      {
        result.push_back(classes[class_name][type]);
      }
    }
    result.push_back(attributes(element));
    return result;
  }

  std::string resolve_path(const std::string& file)
  {
    namespace fs = std::filesystem;
    fs::path path(file);
    if (path.is_absolute() && !compiler.strippath)
    {
      return path.string();
    }
    if (compiler.strippath)
    {
      path = path.filename();
    }
    fs::path directory(compiler.meshdir);
    if (directory.is_absolute())
    {
      return (directory / path).string();
    }
    return (fs::path(filename).parent_path() / directory / path).string();
  }

  void parse_asset(const tinyxml2::XMLElement* element)
  {
    for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
    {
      std::string tag = child->Name();
      Attributes a = attributes(child);
      if (tag == "mesh")
      {
        if (!has(a, "file"))
        {
          throw std::runtime_error("MJCF: only meshes with files are supported");
        }
        MJCFMesh mesh;
        std::string name = has(a, "name") ? a["name"] : std::filesystem::path(a["file"]).stem().string();
        mesh.path = resolve_path(a["file"]);
        if (has(a, "scale"))
        {
          mesh.scale = parse_numbers(a["scale"], 3);
        }
        meshes[name] = mesh;
      }
      else if (tag == "material")
      {
        if (!has(a, "name"))
        {
          throw std::runtime_error("MJCF: a material has no name");
        }
        Eigen::Vector4d rgba = Eigen::Vector4d::Ones();
        std::vector<Attributes> sources;
        if (has_main_defaults && main_defaults.count("material"))
        {
          sources.push_back(main_defaults["material"]);
        }
        if (has(a, "class") && classes.count(a["class"]) && classes[a["class"]].count("material"))
        {
          sources.push_back(classes[a["class"]]["material"]);
        }
        sources.push_back(a);
        for (auto& source : sources)
        {
          if (has(source, "rgba"))
          {
            rgba = parse_numbers(source.at("rgba"), 4);
          }
        }
        materials[a["name"]] = rgba;
      }
      else if (tag == "hfield")
      {
        throw std::runtime_error("MJCF: hfields are not supported");
      }
    }
  }

  // Placement given by pos and an orientation (quat, axisangle, euler, xyaxes or zaxis)
  Eigen::Affine3d parse_placement(const Attributes& a)
  {
    Eigen::Affine3d placement = Eigen::Affine3d::Identity();
    if (has(a, "pos"))
    {
      placement.translation() = parse_numbers(a.at("pos"), 3);
    }
    if (has(a, "quat"))
    {
      Eigen::Vector4d q = parse_numbers(a.at("quat"), 4);
      Eigen::Quaterniond quaternion(q[0], q[1], q[2], q[3]);
      quaternion.normalize();
      placement.linear() = quaternion.toRotationMatrix();
    }
    else if (has(a, "axisangle"))
    {
      Eigen::Vector4d axis_angle = parse_numbers(a.at("axisangle"), 4);
      placement.linear() =
          Eigen::AngleAxisd(compiler.angle(axis_angle[3]), Eigen::Vector3d(axis_angle.head<3>())).toRotationMatrix();
    }
    else if (has(a, "euler"))
    {
      Eigen::Vector3d angles = parse_numbers(a.at("euler"), 3);
      Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
      for (int k = 0; k < 3; k++)
      {
        R = R * Eigen::AngleAxisd(compiler.angle(angles[k]), Eigen::Vector3d(compiler.euler_axes.col(k)))
                    .toRotationMatrix();
      }
      placement.linear() = R;
    }
    else if (has(a, "xyaxes"))
    {
      Eigen::VectorXd axes = parse_numbers(a.at("xyaxes"), 6);
      Eigen::Vector3d x = axes.head<3>().normalized();
      Eigen::Vector3d y = axes.tail<3>();
      y -= x * x.dot(y);
      y.normalize();
      Eigen::Vector3d z = x.cross(y).normalized();
      placement.linear().col(0) = x;
      placement.linear().col(1) = y;
      placement.linear().col(2) = z;
    }
    else if (has(a, "zaxis"))
    {
      Eigen::Vector3d z = parse_numbers(a.at("zaxis"), 3);
      placement.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), z).toRotationMatrix();
    }
    return placement;
  }

  Inertia parse_inertial(const tinyxml2::XMLElement* element)
  {
    Attributes a = attributes(element);
    if (!has(a, "mass"))
    {
      throw std::runtime_error("MJCF: an inertial has no mass");
    }
    double mass = std::max(parse_numbers(a["mass"], 1)[0], compiler.bound_mass);
    if (mass < 0)
    {
      throw std::runtime_error("MJCF: negative mass");
    }
    Eigen::Vector3d com = has(a, "pos") ? Eigen::Vector3d(parse_numbers(a["pos"], 3)) : Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = parse_placement(a).linear();

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    if (has(a, "diaginertia"))
    {
      I = Eigen::Vector3d(parse_numbers(a["diaginertia"], 3)).asDiagonal();
    }
    else if (has(a, "fullinertia"))
    {
      Eigen::VectorXd values = parse_numbers(a["fullinertia"], 6);
      I << values[0], values[3], values[4], values[3], values[1], values[5], values[4], values[5], values[2];
    }
    for (int k = 0; k < 3; k++)
    {
      I(k, k) = std::max(I(k, k), compiler.bound_inertia);
    }

    return Inertia(mass, com, R * I * R.transpose());
  }

  MJCFJoint parse_joint(const tinyxml2::XMLElement* element, const MJCFBody& body)
  {
    MJCFJoint joint;
    joint.name = element->Attribute("name") ? element->Attribute("name") :
                                              body.name + "Joint_" + std::to_string(body.joints.size());
    Attributes own = attributes(element);
    bool use_limits = compiler.autolimits || (has(own, "limited") && own["limited"] == "true");
    joint.placement = parse_placement(own);

    for (auto& a : element_attributes(element, "joint", body.child_class))
    {
      if (!use_limits && has(a, "range"))
      {
        throw std::runtime_error("MJCF: range limit of joint " + joint.name + " is specified but not used");
      }
      if (has(a, "type"))
      {
        joint.type = a.at("type");
      }
      if (has(a, "axis"))
      {
        joint.axis = parse_numbers(a.at("axis"), 3);
      }
      bool has_range = false;
      if (has(a, "range"))
      {
        Eigen::Vector2d range = parse_numbers(a.at("range"), 2);
        // Ranges are angles for hinge joints, and lengths for slide joints
        double converter = joint.type == "slide" ? 1. : compiler.angle_converter;
        joint.min_config = range[0] * converter;
        joint.max_config = range[1] * converter;
        has_range = true;
      }
      if (has(a, "actuatorfrcrange"))
      {
        Eigen::Vector2d range = parse_numbers(a.at("actuatorfrcrange"), 2);
        joint.max_effort = range[1];
      }
      if (has(a, "armature"))
      {
        joint.armature = parse_numbers(a.at("armature"), 1)[0];
      }
      if (has(a, "ref"))
      {
        double value = parse_numbers(a.at("ref"), 1)[0];
        if (joint.type == "slide")
        {
          joint.ref = value;
        }
        else if (joint.type == "hinge")
        {
          joint.ref = compiler.angle(value);
        }
        else
        {
          throw std::runtime_error("MJCF: reference position can only be used with hinge or slide joints");
        }
        if (has_range)
        {
          joint.min_config -= joint.ref;
          joint.max_config -= joint.ref;
        }
      }
    }

    return joint;
  }

  // Geometry kind, size and inertia
  MJCFGeom parse_geom(const tinyxml2::XMLElement* element, const MJCFBody& body)
  {
    MJCFGeom geom;
    geom.name = element->Attribute("name") ? element->Attribute("name") :
                                             body.name + "Geom_" + std::to_string(body.geoms.size());

    for (auto& a : element_attributes(element, "geom", body.child_class))
    {
      if (has(a, "pos") && has(a, "fromto"))
      {
        throw std::runtime_error("MJCF: both pos and fromto are defined for geom " + geom.name);
      }
      geom.placement = parse_placement(a);
      if (has(a, "density"))
      {
        geom.density = parse_numbers(a.at("density"), 1)[0];
      }
      if (has(a, "contype"))
      {
        geom.contype = std::stoi(a.at("contype"));
      }
      if (has(a, "conaffinity"))
      {
        geom.conaffinity = std::stoi(a.at("conaffinity"));
      }
      if (has(a, "group"))
      {
        geom.group = std::stoi(a.at("group"));
      }
      geom.has_mass = has(a, "mass");
      if (geom.has_mass)
      {
        geom.mass = parse_numbers(a.at("mass"), 1)[0];
      }
      if (has(a, "shellinertia"))
      {
        throw std::runtime_error("MJCF: shell inertia is not supported");
      }
      if (has(a, "type"))
      {
        geom.type = a.at("type");
      }
      if (has(a, "material"))
      {
        geom.material = a.at("material");
      }
      if (geom.type == "mesh" && has(a, "mesh"))
      {
        geom.mesh = a.at("mesh");
      }
      if (has(a, "rgba"))
      {
        geom.rgba = parse_numbers(a.at("rgba"), 4);
      }
      if (has(a, "size"))
      {
        geom.size_text = a.at("size");
      }
      if (geom.fromto == "" && has(a, "fromto"))
      {
        geom.fromto = a.at("fromto");
      }
      if (geom.fromto != "")
      {
        Eigen::VectorXd points = parse_numbers(geom.fromto, 6);
        geom.placement.translation() = (points.head<3>() + points.tail<3>()) / 2;
        Eigen::Vector3d z = points.tail<3>() - points.head<3>();
        geom.placement.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), z).toRotationMatrix();
      }
    }

    if (geom.type == "mesh" && geom.mesh == "")
    {
      throw std::runtime_error("MJCF: geom " + geom.name + " is a mesh but has no mesh");
    }

    // Kind
    if (geom.contype == 0 && geom.conaffinity == 0)
    {
      geom.kind = MJCFGeom::VISUAL;
    }
    else if (geom.group > 2)
    {
      geom.kind = MJCFGeom::COLLISION;
    }

    // Size
    double fromto_length = 0.;
    if (geom.fromto != "")
    {
      if (geom.type == "plane" || geom.type == "hfield" || geom.type == "mesh" || geom.type == "sphere")
      {
        throw std::runtime_error("MJCF: fromto can only be used with capsules, boxes, cylinders and ellipsoids");
      }
      Eigen::VectorXd points = parse_numbers(geom.fromto, 6);
      fromto_length = (points.tail<3>() - points.head<3>()).norm();
    }
    if (geom.type == "sphere")
    {
      geom.size = parse_numbers(geom.size_text, 1);
    }
    else if (geom.type == "plane")
    {
      geom.size = parse_numbers(geom.size_text, 3);
    }
    else if (geom.type == "box")
    {
      if (geom.fromto == "")
      {
        geom.size = parse_numbers(geom.size_text, 3) * 2;
      }
      else
      {
        double side = parse_numbers(geom.size_text, 1)[0] * 2;
        geom.size = Eigen::Vector3d(side, side, fromto_length);
      }
    }
    else if (geom.type == "ellipsoid")
    {
      if (geom.fromto == "")
      {
        geom.size = parse_numbers(geom.size_text, 3);
      }
      else
      {
        double radius = parse_numbers(geom.size_text, 1)[0];
        geom.size = Eigen::Vector3d(radius, radius, fromto_length / 2);
      }
    }
    else if (geom.type == "cylinder" || geom.type == "capsule")
    {
      if (geom.fromto == "")
      {
        geom.size = parse_numbers(geom.size_text, 2);
      }
      else
      {
        geom.size = Eigen::Vector2d(parse_numbers(geom.size_text, 1)[0], fromto_length / 2);
      }
    }
    else if (geom.type != "mesh")
    {
      throw std::runtime_error("MJCF: geom type " + geom.type + " is not supported");
    }

    // Inertia
    if (geom.type != "mesh" && geom.type != "plane")
    {
      geom.inertia = primitive_inertia(geom);
    }

    return geom;
  }

  static Inertia primitive_inertia(const MJCFGeom& geom)
  {
    const Eigen::VectorXd& size = geom.size;
    double volume = 0.;
    if (geom.type == "box")
    {
      volume = size.prod();
    }
    else if (geom.type == "cylinder")
    {
      volume = 2 * M_PI * std::pow(size(0), 2) * size(1);
    }
    else if (geom.type == "sphere")
    {
      volume = 4.0 / 3 * M_PI * std::pow(size(0), 3);
    }
    else if (geom.type == "capsule")
    {
      volume = 4.0 / 3 * M_PI * std::pow(size(0), 3) + 2 * M_PI * std::pow(size(0), 2) * size(1);
    }
    else if (geom.type == "ellipsoid")
    {
      volume = 4.0 / 3 * M_PI * size.prod();
    }
    const double mass = geom.has_mass ? geom.mass : volume * geom.density;

    Eigen::Vector3d diagonal;
    if (geom.type == "box")
    {
      const double x = size(0), y = size(1), z = size(2);
      diagonal << mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (y * y + x * x) / 12;
    }
    else if (geom.type == "cylinder")
    {
      const double radius_square = size(0) * size(0), length = size(1) * 2;
      const double a = mass * (radius_square / 4 + length * length / 12);
      diagonal << a, a, mass * (radius_square / 2);
    }
    else if (geom.type == "sphere" || geom.type == "ellipsoid")
    {
      const double x = size(0);
      const double y = geom.type == "sphere" ? size(0) : size(1);
      const double z = geom.type == "sphere" ? size(0) : size(2);
      diagonal << mass * (y * y + z * z) / 5, mass * (x * x + z * z) / 5, mass * (y * y + x * x) / 5;
    }
    else
    {
      // Capsule: a cylinder and two half spheres
      const double radius = size(0), height = size(1) * 2;
      const double v_cyl = M_PI * std::pow(radius, 2) * height;
      const double v_hs = 2. / 3 * std::pow(radius, 3) * M_PI;
      const double total_v = v_cyl + 2 * v_hs;
      const double m_cyl = mass * (v_cyl / total_v);
      const double m_hs = mass * (v_hs / total_v);
      const double dist_hc_cyl = height / 2 + 0.375 * radius;
      const double ix_c = m_cyl * (std::pow(height, 2) / 12 + std::pow(radius, 2) / 4);
      const double iz_c = m_cyl * std::pow(radius, 2) / 2;
      const double ix_hs = m_hs * std::pow(radius, 2) * 0.259375;
      const double iz_hs = m_hs * std::pow(radius, 2) * 0.4;
      const double ix = ix_c + 2 * (ix_hs + m_hs * std::pow(dist_hc_cyl, 2));
      diagonal << ix, ix, iz_c + 2 * iz_hs;
    }

    return Inertia(mass, Eigen::Vector3d::Zero(), diagonal.asDiagonal());
  }

  MJCFSite parse_site(const tinyxml2::XMLElement* element, const MJCFBody& body)
  {
    MJCFSite site;
    site.name = element->Attribute("name") ? element->Attribute("name") :
                                             body.name + "Site_" + std::to_string(body.sites.size());
    for (auto& a : element_attributes(element, "site", body.child_class))
    {
      if (has(a, "pos") && has(a, "fromto"))
      {
        throw std::runtime_error("MJCF: both pos and fromto are defined for site " + site.name);
      }
      site.placement = parse_placement(a);
      if (has(a, "fromto"))
      {
        Eigen::VectorXd points = parse_numbers(a.at("fromto"), 6);
        site.placement.translation() = (points.head<3>() + points.tail<3>()) / 2;
        Eigen::Vector3d z = points.tail<3>() - points.head<3>();
        site.placement.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), z).toRotationMatrix();
      }
    }
    return site;
  }

  void parse_body(const tinyxml2::XMLElement* element, const std::string& child_class, const std::string& parent)
  {
    MJCFBody body;
    body.name = element->Attribute("name") ? element->Attribute("name") : parent + "Bis";
    body.parent = parent;
    body.placement = parse_placement(attributes(element));
    body.child_class = element->Attribute("childclass") ? element->Attribute("childclass") : child_class;
    bodies_order.push_back(body.name);

    const bool has_inertial = element->FirstChildElement("inertial") != nullptr;
    if (compiler.inertia_from_geom == 0 && !has_inertial)
    {
      throw std::runtime_error("MJCF: body " + body.name + " has no inertial, and inertia from geom is disabled");
    }
    const bool geom_inertia = compiler.inertia_from_geom == 1 || (compiler.inertia_from_geom < 0 && !has_inertial);

    for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
    {
      std::string tag = child->Name();
      if (tag == "inertial" && !geom_inertia)
      {
        body.inertia = parse_inertial(child);
      }
      else if (tag == "geom")
      {
        body.geoms.push_back(parse_geom(child, body));
      }
      else if (tag == "joint")
      {
        body.joints.push_back(parse_joint(child, body));
      }
      else if (tag == "freejoint")
      {
        MJCFJoint joint;
        joint.type = "free";
        joint.name = child->Attribute("name") ? child->Attribute("name") : body.name + "_free";
        body.joints.push_back(joint);
      }
      else if (tag == "site")
      {
        body.sites.push_back(parse_site(child, body));
      }
      else if (tag == "body")
      {
        parse_body(child, body.child_class, body.name);
      }
    }

    if (geom_inertia)
    {
      body.inertia = Inertia();
      for (auto& geom : body.geoms)
      {
        if (geom.kind != MJCFGeom::VISUAL)
        {
          body.inertia += geom.inertia.transformed(geom.placement);
        }
      }
    }

    bodies[body.name] = body;
  }

  int body_frame(const std::string& name)
  {
    int frame = model.get_frame_id(name, Frame::BODY);
    if (frame >= (int)model.frames.size())
    {
      throw std::runtime_error("MJCF: no frame for body " + name);
    }
    return frame;
  }

  void append_reference(const Eigen::VectorXd& q)
  {
    reference_configuration.conservativeResize(reference_configuration.size() + q.size());
    reference_configuration.tail(q.size()) = q;
  }

  void add_body(const std::string& name)
  {
    const MJCFBody& body = bodies.at(name);
    const int parent_frame_id = body.parent == "" ? 0 : body_frame(body.parent);
    const Frame parent_frame = model.frames[parent_frame_id];
    const double infinity = std::numeric_limits<double>::infinity();

    if (body.joints.size() == 0)
    {
      if (body.parent == "")
      {
        return;
      }

      // Fixed body (merged with its parent)
      Frame frame;
      frame.name = name + "_fixed";
      frame.parent_joint = parent_frame.parent_joint;
      frame.parent_frame = parent_frame_id;
      frame.placement = parent_frame.placement * body.placement;
      frame.type = Frame::FIXED_JOINT;
      frame.inertia = body.inertia;
      int fixed_frame = model.add_frame(frame);
      int body_frame_id = model.add_body_frame(name, frame.parent_joint, frame.placement, fixed_frame);

      for (auto& site : body.sites)
      {
        add_site(site, body_frame_id, model.frames[body_frame_id].placement * site.placement);
      }
      return;
    }

    if (body.joints.size() > 1)
    {
      throw std::runtime_error("MJCF: body " + name + " has several joints, which is not supported");
    }

    const MJCFJoint& mjcf_joint = body.joints[0];
    const Eigen::Affine3d joint_in_parent = body.placement * mjcf_joint.placement;
    const Eigen::Affine3d body_in_joint = inverse(mjcf_joint.placement);

    Joint joint;
    Eigen::VectorXd max_effort, max_velocity, min_config, max_config, armature;
    if (mjcf_joint.type == "free")
    {
      joint = Joint(Joint::FREE_FLYER);
      max_effort = max_velocity = Eigen::VectorXd::Constant(6, infinity);
      min_config = Eigen::VectorXd::Constant(7, -1.01);
      max_config = Eigen::VectorXd::Constant(7, 1.01);
      armature = Eigen::VectorXd::Constant(6, mjcf_joint.armature);

      // The free joint configuration is the pose of the body
      const Eigen::Quaterniond quaternion(body.placement.linear());
      const Eigen::Quaterniond inverse_quaternion = quaternion.inverse();
      const Eigen::Vector3d t = -body.placement.translation();
      Eigen::VectorXd reference(7);
      reference << t(0), t(1), t(2), inverse_quaternion.x(), inverse_quaternion.y(), inverse_quaternion.z(),
          inverse_quaternion.w();
      append_reference(reference);
    }
    else if (mjcf_joint.type == "ball")
    {
      joint = Joint(Joint::SPHERICAL);
      max_effort = max_velocity = Eigen::VectorXd::Constant(3, infinity);
      min_config = Eigen::VectorXd::Constant(4, -1.01);
      max_config = Eigen::VectorXd::Constant(4, 1.01);
      armature = Eigen::VectorXd::Constant(3, mjcf_joint.armature);
      append_reference(Eigen::Vector4d(0, 0, 0, 1));
    }
    else if (mjcf_joint.type == "hinge" || mjcf_joint.type == "slide")
    {
      joint = Joint(mjcf_joint.type == "hinge" ? Joint::REVOLUTE : Joint::PRISMATIC, mjcf_joint.axis);
      max_effort = Eigen::VectorXd::Constant(1, mjcf_joint.max_effort);
      max_velocity = Eigen::VectorXd::Constant(1, infinity);
      min_config = Eigen::VectorXd::Constant(1, mjcf_joint.min_config);
      max_config = Eigen::VectorXd::Constant(1, mjcf_joint.max_config);
      armature = Eigen::VectorXd::Constant(1, mjcf_joint.armature);
      append_reference(Eigen::VectorXd::Constant(1, -mjcf_joint.ref));
    }
    else
    {
      throw std::runtime_error("MJCF: unknown joint type " + mjcf_joint.type);
    }

    int joint_id = model.add_joint(parent_frame.parent_joint, joint, parent_frame.placement * joint_in_parent,
                                   mjcf_joint.name, max_effort, max_velocity, min_config, max_config);
    model.armature.segment(model.joints[joint_id].idx_v, model.joints[joint_id].nv) = armature;
    int joint_frame = model.add_joint_frame(joint_id, parent_frame_id);

    // Body attached to the joint
    const Eigen::Affine3d placement = model.frames[joint_frame].placement * body_in_joint;
    if (!body.inertia.is_zero())
    {
      model.append_body_to_joint(joint_id, body.inertia, placement);
    }
    int body_frame_id = model.add_body_frame(name, joint_id, placement, joint_frame);

    for (auto& site : body.sites)
    {
      add_site(site, body_frame_id, body_in_joint * site.placement);
    }
  }

  void add_site(const MJCFSite& site, int body_frame_id, const Eigen::Affine3d& placement)
  {
    Frame frame;
    frame.name = site.name;
    frame.parent_joint = model.frames[body_frame_id].parent_joint;
    frame.parent_frame = body_frame_id;
    frame.placement = placement;
    frame.type = Frame::OP_FRAME;
    model.add_frame(frame);
  }

  void build_model()
  {
    const MJCFBody& root = bodies.at(bodies_order[0]);
    if (root.joints.size() == 0)
    {
      // The robot is attached to the world with a free-flyer
      int root_joint = model.add_joint(0, Joint(Joint::FREE_FLYER), Eigen::Affine3d::Identity(), "root_joint");
      int root_joint_frame = model.add_joint_frame(root_joint, 0);
      if (!root.inertia.is_zero())
      {
        model.append_body_to_joint(root_joint, root.inertia, Eigen::Affine3d::Identity());
      }
      model.add_body_frame(root.name, root_joint, Eigen::Affine3d::Identity(), root_joint_frame);
      append_reference(model.neutral());
    }

    for (auto& name : bodies_order)
    {
      add_body(name);
    }

    // The joints placements correspond to the reference configuration (joints "ref" attributes)
    Data data(model);
    forward_kinematics(model, data, reference_configuration);
    for (int i = 1; i < model.njoints; i++)
    {
      model.joint_placements[i] = inverse(data.oMi[model.parents[i]]) * data.oMi[i];
    }
  }

  void build_geometries(GeometryModel& geometry_model, bool visual)
  {
    add_geometries(world, 0, geometry_model, visual);
    for (auto& name : bodies_order)
    {
      add_geometries(bodies.at(name), body_frame(name), geometry_model, visual);
    }
  }

  void add_geometries(const MJCFBody& body, int frame_id, GeometryModel& geometry_model, bool visual)
  {
    const Frame& frame = model.frames[frame_id];
    for (auto& geom : body.geoms)
    {
      if (visual ? geom.kind == MJCFGeom::COLLISION : geom.kind == MJCFGeom::VISUAL)
      {
        continue;
      }

      GeometryObject object;
      object.name = geom.name;
      object.parent_joint = frame.parent_joint;
      object.parent_frame = frame_id;
      object.placement = frame.placement * geom.placement;
      object.color = geom.rgba;

      if (geom.type == "mesh")
      {
        if (!meshes.count(geom.mesh))
        {
          throw std::runtime_error("MJCF: can't find mesh " + geom.mesh + " for geom " + geom.name);
        }
        object.type = GeometryObject::MESH;
        object.mesh_path = meshes[geom.mesh].path;
        object.mesh_scale = meshes[geom.mesh].scale;
      }
      else if (geom.type == "box")
      {
        object.type = GeometryObject::BOX;
        object.size = geom.size;
      }
      else if (geom.type == "sphere")
      {
        object.type = GeometryObject::SPHERE;
        object.size = Eigen::Vector3d(geom.size(0), 0., 0.);
      }
      else if (geom.type == "cylinder" || geom.type == "capsule")
      {
        object.type = geom.type == "cylinder" ? GeometryObject::CYLINDER : GeometryObject::CAPSULE;
        object.size = Eigen::Vector3d(geom.size(0), geom.size(1) * 2, 0.);
      }
      else if (geom.type == "ellipsoid")
      {
        object.type = GeometryObject::ELLIPSOID;
        object.size = geom.size;
      }
      else if (geom.type == "plane")
      {
        object.type = GeometryObject::PLANE;
        object.size = geom.size;
      }

      if (geom.material != "")
      {
        if (!materials.count(geom.material))
        {
          throw std::runtime_error("MJCF: can't find material " + geom.material + " for geom " + geom.name);
        }
        object.color = materials[geom.material];
        object.has_color = true;
      }

      geometry_model.add_object(object);
    }
  }
};
}  // namespace

void load_mjcf(const std::string& filename, Model& model, GeometryModel& collision_model, GeometryModel& visual_model,
               bool collision_as_visual)
{
  MJCFLoader loader(filename, model, collision_model, visual_model, collision_as_visual);
  loader.load();
}
}  // namespace placo::model
