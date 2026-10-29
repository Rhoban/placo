#include "placo/model/model.h"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace placo::model
{
Model::Model()
{
  gravity = Eigen::Vector3d(0., 0., -9.81);

  // The universe (joint 0)
  joints.push_back(Joint(Joint::UNIVERSE));
  names.push_back("universe");
  parents.push_back(0);
  joint_placements.push_back(Eigen::Affine3d::Identity());
  inertias.push_back(Inertia());
  supports.push_back({ 0 });
  subtrees.push_back({ 0 });
  njoints = 1;

  Frame universe;
  universe.name = "universe";
  universe.type = Frame::FIXED_JOINT;
  frames.push_back(universe);
}

int Model::add_joint(int parent, const Joint& joint_, const Eigen::Affine3d& placement, const std::string& name,
                     const Eigen::VectorXd& max_effort, const Eigen::VectorXd& max_velocity,
                     const Eigen::VectorXd& min_config, const Eigen::VectorXd& max_config)
{
  if (parent < 0 || parent >= njoints)
  {
    throw std::runtime_error("Model::add_joint: invalid parent joint for " + name);
  }

  Joint joint = joint_;
  joint.idx_q = nq;
  joint.idx_v = nv;

  if (max_effort.size() != joint.nv || max_velocity.size() != joint.nv || min_config.size() != joint.nq ||
      max_config.size() != joint.nq)
  {
    throw std::runtime_error("Model::add_joint: limits of joint " + name + " don't have the right size");
  }

  int id = njoints++;
  joints.push_back(joint);
  names.push_back(name);
  parents.push_back(parent);
  joint_placements.push_back(placement);
  inertias.push_back(Inertia());

  supports.push_back(supports[parent]);
  supports.back().push_back(id);
  subtrees.push_back({ id });
  for (int ancestor : supports[parent])
  {
    subtrees[ancestor].push_back(id);
  }

  // Limits
  lower_position_limit.conservativeResize(nq + joint.nq);
  lower_position_limit.tail(joint.nq) = min_config;
  upper_position_limit.conservativeResize(nq + joint.nq);
  upper_position_limit.tail(joint.nq) = max_config;
  effort_limit.conservativeResize(nv + joint.nv);
  effort_limit.tail(joint.nv) = max_effort;
  velocity_limit.conservativeResize(nv + joint.nv);
  velocity_limit.tail(joint.nv) = max_velocity;
  armature.conservativeResize(nv + joint.nv);
  armature.tail(joint.nv).setZero();
  rotor_inertia.conservativeResize(nv + joint.nv);
  rotor_inertia.tail(joint.nv).setZero();
  rotor_gear_ratio.conservativeResize(nv + joint.nv);
  rotor_gear_ratio.tail(joint.nv).setOnes();

  nq += joint.nq;
  nv += joint.nv;

  return id;
}

int Model::add_joint(int parent, const Joint& joint, const Eigen::Affine3d& placement, const std::string& name)
{
  const double max = std::numeric_limits<double>::max();
  return add_joint(parent, joint, placement, name, Eigen::VectorXd::Constant(joint.nv, max),
                   Eigen::VectorXd::Constant(joint.nv, max), Eigen::VectorXd::Constant(joint.nq, -max),
                   Eigen::VectorXd::Constant(joint.nq, max));
}

int Model::add_frame(const Frame& frame, bool append_inertia)
{
  if (frame.parent_joint < 0 || frame.parent_joint >= njoints)
  {
    throw std::runtime_error("Model::add_frame: invalid parent joint for frame " + frame.name);
  }

  if (exist_frame(frame.name, frame.type))
  {
    return get_frame_id(frame.name, frame.type);
  }

  frames.push_back(frame);
  if (append_inertia && frame.inertia.mass > 0.)
  {
    inertias[frame.parent_joint] += frame.inertia.transformed(frame.placement);
  }

  return frames.size() - 1;
}

int Model::add_joint_frame(int joint, int parent_frame)
{
  Frame frame;
  frame.name = names[joint];
  frame.parent_joint = joint;
  frame.parent_frame = parent_frame;
  frame.type = Frame::JOINT;
  return add_frame(frame);
}

int Model::add_body_frame(const std::string& name, int parent_joint, const Eigen::Affine3d& placement, int parent_frame)
{
  Frame frame;
  frame.name = name;
  frame.parent_joint = parent_joint;
  frame.parent_frame = parent_frame;
  frame.placement = placement;
  frame.type = Frame::BODY;
  return add_frame(frame);
}

void Model::append_body_to_joint(int joint, const Inertia& inertia, const Eigen::Affine3d& placement)
{
  inertias[joint] += inertia.transformed(placement);
}

bool Model::exist_frame(const std::string& name, int type) const
{
  return get_frame_id(name, type) < (int)frames.size();
}

int Model::get_frame_id(const std::string& name, int type) const
{
  for (int k = 0; k < (int)frames.size(); k++)
  {
    if (frames[k].name == name && (frames[k].type & type))
    {
      return k;
    }
  }
  return frames.size();
}

bool Model::exist_joint(const std::string& name) const
{
  return std::find(names.begin(), names.end(), name) != names.end();
}

int Model::get_joint_id(const std::string& name) const
{
  auto it = std::find(names.begin(), names.end(), name);
  if (it == names.end())
  {
    throw std::runtime_error("Joint with name " + name + " not found in model");
  }
  return it - names.begin();
}

Eigen::VectorXd Model::neutral() const
{
  Eigen::VectorXd q(nq);
  for (auto& joint : joints)
  {
    joint.neutral(q);
  }
  return q;
}

Eigen::VectorXd Model::integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const
{
  Eigen::VectorXd q_out(nq);
  for (auto& joint : joints)
  {
    joint.integrate(q, v, q_out);
  }
  return q_out;
}

Eigen::VectorXd Model::difference(const Eigen::VectorXd& q0, const Eigen::VectorXd& q1) const
{
  Eigen::VectorXd d(nv);
  for (auto& joint : joints)
  {
    joint.difference(q0, q1, d);
  }
  return d;
}

Eigen::VectorXd Model::random_configuration() const
{
  Eigen::VectorXd q(nq);
  for (auto& joint : joints)
  {
    joint.random(lower_position_limit, upper_position_limit, q);
  }
  return q;
}

double Model::total_mass() const
{
  double mass = 0.;
  for (auto& inertia : inertias)
  {
    mass += inertia.mass;
  }
  return mass;
}
}  // namespace placo::model
