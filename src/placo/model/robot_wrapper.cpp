#include "placo/model/robot_wrapper.h"
#include "placo/model/algorithms.h"
#include "placo/model/urdf.h"
#include "placo/model/mjcf.h"
#include "placo/tools/utils.h"
#include <json/json.h>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <numeric>

namespace fs = std::filesystem;

namespace placo::model
{
RobotWrapper::RobotWrapper(std::string model_directory, int flags, std::string urdf_content)
  : model_directory(model_directory)
{
  std::string urdf_filename;

  if (fs::is_regular_file(model_directory))
  {
    fs::path path = model_directory;
    urdf_filename = model_directory;
    model_directory = path.parent_path().string();
  }
  else
  {
    if (flags & Flags::MJCF)
    {
      urdf_filename = model_directory + "/robot.xml";
    }
    else
    {
      urdf_filename = model_directory + "/robot.urdf";
    }
  }

  if (flags & Flags::MJCF)
  {
    if (urdf_content != "")
    {
      throw std::runtime_error("Loading MJCF with direct contents is not supported");
    }
    std::cout << "Loading model: " << urdf_filename << std::endl;
    load_mjcf(urdf_filename, model, collision_model, visual_model, flags & COLLISION_AS_VISUAL);
  }
  else
  {
    if (urdf_content == "")
    {
      std::ifstream file(urdf_filename);
      if (!file.is_open())
      {
        throw std::runtime_error("Can't open URDF file " + urdf_filename);
      }
      std::stringstream buffer;
      buffer << file.rdbuf();
      urdf_content = buffer.str();
    }
    load_urdf(urdf_content, model, collision_model, visual_model, { model_directory }, flags & COLLISION_AS_VISUAL);
  }
  collision_model.build_collision_geometries();

  // Load collisions pairs
  if (!(flags & IGNORE_COLLISIONS))
  {
    if (tools::file_exists(model_directory + "/collisions.json"))
    {
      load_collision_pairs(model_directory + "/collisions.json");
    }
    else
    {
      collision_model.add_all_collision_pairs();
    }
  }

  // Creating data
  data = new Data(model);
  all_columns.resize(model.nv);
  std::iota(all_columns.begin(), all_columns.end(), 0);

  // Configuration coordinate of each degree of freedom (see v_to_q)
  v_to_q.assign(model.nv, -1);
  for (int j = 1; j < model.njoints; j++)
  {
    const auto& joint = model.joints[j];
    if (joint.nq == joint.nv && joint.idx_v >= 0)
    {
      for (int i = 0; i < joint.nv; i++)
      {
        v_to_q[joint.idx_v + i] = joint.idx_q + i;
      }
    }
  }

  // Assuming that motors with limits both equals to zero are not defined in the
  // URDF, setting them to the maximum possible value
  for (int k = 0; k < model.nq; k++)
  {
    if (model.lower_position_limit[k] == 0 && model.upper_position_limit[k] == 0)
    {
      model.lower_position_limit[k] = std::numeric_limits<double>::lowest();
      model.upper_position_limit[k] = std::numeric_limits<double>::max();
    }
  }

  reset();
  update_kinematics();

  auto collisions = self_collisions();
  if (collisions.size() > 0)
  {
    std::cerr << "WARNING: Robot has the following self collisions in neutral position:" << std::endl;

    for (auto& collision : collisions)
    {
      std::cerr << "  -" << collision.bodyA << " collides with " << collision.bodyB << std::endl;
    }
  }
}

bool RobotWrapper::Collision::operator==(const Collision& other)
{
  return (objA == other.objA && objB == other.objB);
}

bool RobotWrapper::Distance::operator==(const Distance& other)
{
  return (objA == other.objA && objB == other.objB);
}

void RobotWrapper::reset()
{
  state = neutral_state();
}

RobotWrapper::FrameIndex RobotWrapper::get_frame_index(const std::string& frame)
{
  if (!model.exist_frame(frame))
  {
    std::ostringstream oss;
    oss << "Frame with name " << frame << " not found in model";
    throw std::runtime_error(oss.str());
  }

  return model.get_frame_id(frame);
}

// A continuous joint (unbounded revolute) has a configuration of two coordinates (cos and sin of its angle) for one
// degree of freedom
static bool is_continuous(const Joint& joint)
{
  return joint.nq == 2 && joint.nv == 1;
}

void RobotWrapper::set_joint(const std::string& name, double value)
{
  int k = get_joint_offset(name);
  if (is_continuous(model.joints[model.get_joint_id(name)]))
  {
    state.q[k] = cos(value);
    state.q[k + 1] = sin(value);
  }
  else
  {
    state.q[k] = value;
  }
}

double RobotWrapper::get_joint(const std::string& name)
{
  int k = get_joint_offset(name);
  if (is_continuous(model.joints[model.get_joint_id(name)]))
  {
    return atan2(state.q[k + 1], state.q[k]);
  }
  return state.q[k];
}

int RobotWrapper::get_joint_offset(const std::string& name)
{
  if (!model.exist_joint(name))
  {
    std::ostringstream oss;
    oss << "Joint with name " << name << " not found in model";
    throw std::runtime_error(oss.str());
  }

  auto index = model.get_joint_id(name);
  return model.joints[index].idx_q;
}

int RobotWrapper::get_joint_size(const std::string& name)
{
  auto index = model.get_joint_id(name);
  return model.joints[index].nq;
}

int RobotWrapper::get_joint_v_offset(const std::string& name)
{
  auto index = model.get_joint_id(name);
  return model.joints[index].idx_v;
}

int RobotWrapper::get_joint_v_size(const std::string& name)
{
  auto index = model.get_joint_id(name);
  return model.joints[index].nv;
}

double RobotWrapper::get_joint_velocity(const std::string& name)
{
  return state.qd[get_joint_v_offset(name)];
}

void RobotWrapper::set_joint_acceleration(const std::string& name, double value)
{
  state.qdd[get_joint_v_offset(name)] = value;
}

double RobotWrapper::get_joint_acceleration(const std::string& name)
{
  return state.qdd[get_joint_v_offset(name)];
}

void RobotWrapper::set_joint_velocity(const std::string& name, double value)
{
  state.qd[get_joint_v_offset(name)] = value;
}

void RobotWrapper::set_velocity_limit(const std::string& name, double limit)
{
  model.velocity_limit[get_joint_v_offset(name)] = limit;
}

void RobotWrapper::set_torque_limit(const std::string& name, double limit)
{
  model.effort_limit[get_joint_v_offset(name)] = limit;
}

void RobotWrapper::set_velocity_limits(double limit)
{
  for (auto& name : joint_names())
  {
    set_velocity_limit(name, limit);
  }
}

void RobotWrapper::set_joint_limits(const std::string& name, double lower, double upper)
{
  if (is_continuous(model.joints[model.get_joint_id(name)]))
  {
    throw std::runtime_error("RobotWrapper::set_joint_limits: " + name + " is a continuous joint (no position limits)");
  }
  int k = get_joint_offset(name);
  model.lower_position_limit[k] = lower;
  model.upper_position_limit[k] = upper;
}

std::pair<double, double> RobotWrapper::get_joint_limits(const std::string& name)
{
  if (is_continuous(model.joints[model.get_joint_id(name)]))
  {
    return std::make_pair(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  }
  int k = get_joint_offset(name);
  return std::make_pair(model.lower_position_limit[k], model.upper_position_limit[k]);
}

Eigen::Affine3d RobotWrapper::get_T_world_fbase()
{
  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();

  transformation.translation() = Eigen::Vector3d(state.q[0], state.q[1], state.q[2]);

  auto rotation = Eigen::Quaterniond(state.q[6], state.q[3], state.q[4], state.q[5]);
  transformation.linear() = rotation.toRotationMatrix();

  // The free-flyer configuration encodes SE3(q), while the actual base placement is
  // oMi[1] = jointPlacements[1] * SE3(q). Account for a non-identity root joint placement.
  return model.joint_placements[1] * transformation;
}

void RobotWrapper::set_T_world_fbase(Eigen::Affine3d transformation)
{
  // Undo the root joint placement so that oMi[1] == transformation (see get_T_world_fbase)
  transformation = inverse(model.joint_placements[1]) * transformation;

  state.q[0] = transformation.translation().x();
  state.q[1] = transformation.translation().y();
  state.q[2] = transformation.translation().z();

  Eigen::Quaterniond quaternions = Eigen::Quaterniond(transformation.linear());
  state.q[3] = quaternions.x();
  state.q[4] = quaternions.y();
  state.q[5] = quaternions.z();
  state.q[6] = quaternions.w();
}

Eigen::Vector3d RobotWrapper::com_world()
{
  center_of_mass(model, *data);

  return data->com;
}

void RobotWrapper::update_kinematics()
{
  // A single forward pass: joint placements and jacobians (and their time variation if needed), then frames
  if (compute_jacobian_time_variation)
  {
    compute_joint_jacobians_time_variation(model, *data, state.q, state.qd);
  }
  else
  {
    compute_joint_jacobians(model, *data, state.q);
  }
  update_frame_placements(model, *data);
}

void RobotWrapper::compute_hessians()
{
  // The hessians are computed on demand from the joint Jacobians (see get_frame_hessian)
}

Eigen::MatrixXd RobotWrapper::get_frame_hessian(FrameIndex frame, int joint_v_index)
{
  return frame_hessian(model, *data, frame, joint_v_index);
}

RobotWrapper::State RobotWrapper::neutral_state()
{
  State state;
  state.q = model.neutral();
  state.qd = Eigen::VectorXd(model.nv);
  state.qd.setZero();
  state.qdd = Eigen::VectorXd(model.nv);
  state.qdd.setZero();

  return state;
}

void RobotWrapper::load_collision_pairs(const std::string& filename)
{
  // Reading collision pairs
  Json::Value collisions;

  // Building a look-up to find geometry objects by name
  std::map<std::string, std::vector<size_t>> name_to_objects;
  for (size_t k = 0; k < collision_model.objects.size(); k++)
  {
    std::string name = model.frames[collision_model.objects[k].parent_frame].name;
    name_to_objects[name].push_back(k);
  }

  collision_model.remove_all_collision_pairs();

  std::ifstream f(filename);
  if (!f.is_open())
  {
    throw std::runtime_error("Can't load collision pairs");
  }
  f >> collisions;

  for (size_t k = 0; k < collisions.size(); k++)
  {
    Json::Value& entry = collisions[(int)k];
    if (entry.size() == 2)
    {
      if (entry[0].isInt() && entry[1].isInt())
      {
        // Entries can be raw pair offset e.g [34, 22]
        size_t k1 = entry[0].asInt();
        size_t k2 = entry[1].asInt();

        collision_model.add_collision_pair(k1, k2);
      }
      else if (entry[0].isString() && entry[1].isString())
      {
        // Entries can be link string names e.g ["trunk", "arm"]
        std::string obj1 = entry[0].asString();
        std::string obj2 = entry[1].asString();

        if (!name_to_objects.count(obj1) || !name_to_objects.count(obj2))
        {
          std::ostringstream oss;
          oss << "Collision pair [" << obj1 << ", " << obj2 << "] can't be loaded (check that bodies exists)";
          throw std::runtime_error(oss.str());
        }

        for (size_t k1 : name_to_objects[obj1])
        {
          for (size_t k2 : name_to_objects[obj2])
          {
            collision_model.add_collision_pair(k1, k2);
          }
        }
      }
      else
      {
        throw std::runtime_error("Collision pairs should be array of int or strings");
      }
    }
  }
}

Eigen::Affine3d RobotWrapper::get_T_world_frame(const std::string& frame)
{
  return get_T_world_frame(get_frame_index(frame));
}

Eigen::Affine3d RobotWrapper::get_T_world_frame(FrameIndex index)
{
  return data->oMf[index];
}

Eigen::Affine3d RobotWrapper::get_T_a_b(const std::string& frame_a, const std::string& frame_b)
{
  return get_T_a_b(get_frame_index(frame_a), get_frame_index(frame_b));
}

Eigen::Affine3d RobotWrapper::get_T_a_b(FrameIndex index_a, FrameIndex index_b)
{
  return get_T_world_frame(index_a).inverse() * get_T_world_frame(index_b);
}

void RobotWrapper::set_T_world_frame(const std::string& frame, Eigen::Affine3d T_world_frameTarget)
{
  set_T_world_frame(get_frame_index(frame), T_world_frameTarget);
}

void RobotWrapper::set_T_world_frame(FrameIndex frame, Eigen::Affine3d T_world_frameTarget)
{
  Eigen::Affine3d T_world_fbase = get_T_world_fbase();
  Eigen::Affine3d T_world_frame = get_T_world_frame(frame);
  Eigen::Affine3d T_frame_fbase = T_world_frame.inverse() * T_world_fbase;

  set_T_world_fbase(T_world_frameTarget * T_frame_fbase);
}

std::vector<RobotWrapper::Collision> RobotWrapper::self_collisions(bool stop_at_first)
{
  std::vector<Collision> collisions;

  forward_kinematics(model, *data, state.q);
  for (auto& result : compute_collisions(collision_model, *data, stop_at_first))
  {
    const auto& pair = collision_model.collision_pairs[result.pair];

    Collision collision;
    collision.objA = pair.first;
    collision.objB = pair.second;
    collision.bodyA = collision_model.objects[pair.first].name;
    collision.parentA = collision_model.objects[pair.first].parent_joint;
    collision.bodyB = collision_model.objects[pair.second].name;
    collision.parentB = collision_model.objects[pair.second].parent_joint;
    collision.contacts = result.contacts;
    collisions.push_back(collision);
  }

  return collisions;
}

std::vector<RobotWrapper::Distance> RobotWrapper::distances()
{
  std::vector<Distance> distances;

  forward_kinematics(model, *data, state.q);
  for (auto& result : compute_distances(collision_model, *data))
  {
    const auto& pair = collision_model.collision_pairs[result.pair];

    Distance distance;
    distance.objA = pair.first;
    distance.objB = pair.second;
    distance.min_distance = result.min_distance;
    distance.pointA = result.point_a;
    distance.pointB = result.point_b;
    distance.parentA = collision_model.objects[pair.first].parent_joint;
    distance.parentB = collision_model.objects[pair.second].parent_joint;
    distances.push_back(distance);
  }

  return distances;
}

static ReferenceFrame string_to_reference(const std::string& reference)
{
  if (reference == "local_world_aligned")
  {
    return LOCAL_WORLD_ALIGNED;
  }
  else if (reference == "local")
  {
    return LOCAL;
  }
  else if (reference == "world")
  {
    return WORLD;
  }
  else
  {
    std::ostringstream oss;
    oss << "Unknown reference: " << reference << ", use one of: world, local, local_world_aligned.";
    throw std::runtime_error(oss.str());
  }
}

Eigen::MatrixXd RobotWrapper::frame_jacobian(const std::string& frame, const std::string& reference)
{
  return frame_jacobian(get_frame_index(frame), string_to_reference(reference));
}

Eigen::MatrixXd RobotWrapper::frame_jacobian(FrameIndex frame, ReferenceFrame ref)
{
  Eigen::MatrixXd jacobian;
  compact_frame_jacobian(frame, ref, all_columns, jacobian);
  return jacobian;
}

Eigen::MatrixXd RobotWrapper::frame_jacobian_time_variation(const std::string& frame, const std::string& reference)
{
  return frame_jacobian_time_variation(get_frame_index(frame), string_to_reference(reference));
}

Eigen::MatrixXd RobotWrapper::frame_jacobian_time_variation(FrameIndex frame, ReferenceFrame ref)
{
  Eigen::MatrixXd jacobian;
  compact_frame_jacobian_time_variation(frame, ref, all_columns, jacobian);
  return jacobian;
}

Eigen::MatrixXd RobotWrapper::joint_jacobian(const std::string& joint, const std::string& reference)
{
  return joint_jacobian(model.get_joint_id(joint), string_to_reference(reference));
}

Eigen::MatrixXd RobotWrapper::joint_jacobian(JointIndex joint, ReferenceFrame ref)
{
  Eigen::MatrixXd jacobian;
  compact_jacobian(joint, data->oMi[joint], ref, all_columns, jacobian);
  return jacobian;
}

Eigen::MatrixXd RobotWrapper::joint_jacobian_time_variation(const std::string& joint, const std::string& reference)
{
  return joint_jacobian_time_variation(model.get_joint_id(joint), string_to_reference(reference));
}

Eigen::MatrixXd RobotWrapper::joint_jacobian_time_variation(JointIndex joint, ReferenceFrame ref)
{
  Eigen::MatrixXd jacobian;
  compact_jacobian_time_variation(joint, data->oMi[joint], ref, all_columns, jacobian);
  return jacobian;
}

Eigen::MatrixXd RobotWrapper::relative_position_jacobian(FrameIndex frame_a, FrameIndex frame_b)
{
  auto T_world_a = get_T_world_frame(frame_a);
  auto T_world_b = get_T_world_frame(frame_b);
  auto T_a_b = T_world_a.inverse() * T_world_b;

  Eigen::MatrixXd R_world_a = T_world_a.linear();

  Eigen::MatrixXd J_a = frame_jacobian(frame_a, LOCAL_WORLD_ALIGNED);
  Eigen::MatrixXd J_b = frame_jacobian(frame_b, LOCAL_WORLD_ALIGNED);

  return (R_world_a.transpose() * (J_b.topRows(3) - J_a.topRows(3)) +
          skew(T_a_b.translation()) * R_world_a.transpose() * J_a.bottomRows(3));
}

const std::vector<int>& RobotWrapper::joint_support(JointIndex joint)
{
  if ((int)joint_supports.size() != model.njoints)
  {
    // Supports are the joints from the universe (index 0, without degrees of freedom) to the given one
    joint_supports.assign(model.njoints, {});
    for (int j = 0; j < model.njoints; j++)
    {
      for (auto support : model.supports[j])
      {
        const Joint& support_joint = model.joints[support];
        for (int k = 0; k < support_joint.nv; k++)
        {
          joint_supports[j].push_back(support_joint.idx_v + k);
        }
      }
      std::sort(joint_supports[j].begin(), joint_supports[j].end());
    }
  }

  return joint_supports[joint];
}

const std::vector<int>& RobotWrapper::frame_support(FrameIndex frame)
{
  return joint_support(model.frames[frame].parent_joint);
}

void RobotWrapper::merge_supports(const std::vector<int>& a, const std::vector<int>& b, std::vector<int>& result)
{
  result.clear();
  std::set_union(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(result));
}

void RobotWrapper::compact_jacobian(JointIndex joint, const Eigen::Affine3d& T_world_point, ReferenceFrame ref,
                                    const std::vector<int>& columns, Eigen::MatrixXd& J)
{
  point_jacobian(model, *data, joint, joint_support(joint), T_world_point, ref, columns, J, false);
}

void RobotWrapper::compact_jacobian_time_variation(JointIndex joint, const Eigen::Affine3d& T_world_point,
                                                   ReferenceFrame ref, const std::vector<int>& columns,
                                                   Eigen::MatrixXd& dJ)
{
  point_jacobian(model, *data, joint, joint_support(joint), T_world_point, ref, columns, dJ, true);
}

void RobotWrapper::compact_frame_jacobian(FrameIndex frame, ReferenceFrame ref, const std::vector<int>& columns,
                                          Eigen::MatrixXd& J)
{
  // The frame placement is updated from the placement of its joint
  const Frame& frame_model = model.frames[frame];
  data->oMf[frame] = data->oMi[frame_model.parent_joint] * frame_model.placement;
  compact_jacobian(frame_model.parent_joint, data->oMf[frame], ref, columns, J);
}

Eigen::MatrixXd RobotWrapper::compact_frame_jacobian(const std::string& frame, const std::string& reference,
                                                     const std::vector<int>& columns)
{
  Eigen::MatrixXd J;
  compact_frame_jacobian(get_frame_index(frame), string_to_reference(reference), columns, J);
  return J;
}

Eigen::MatrixXd RobotWrapper::compact_frame_jacobian_time_variation(const std::string& frame,
                                                                    const std::string& reference,
                                                                    const std::vector<int>& columns)
{
  Eigen::MatrixXd dJ;
  compact_frame_jacobian_time_variation(get_frame_index(frame), string_to_reference(reference), columns, dJ);
  return dJ;
}

void RobotWrapper::compact_frame_jacobian_time_variation(FrameIndex frame, ReferenceFrame ref,
                                                         const std::vector<int>& columns, Eigen::MatrixXd& dJ)
{
  // The frame placement is updated from the placement of its joint
  const Frame& frame_model = model.frames[frame];
  data->oMf[frame] = data->oMi[frame_model.parent_joint] * frame_model.placement;
  compact_jacobian_time_variation(frame_model.parent_joint, data->oMf[frame], ref, columns, dJ);
}

Eigen::MatrixXd RobotWrapper::relative_position_jacobian(const std::string& frame_a, const std::string& frame_b)
{
  return relative_position_jacobian(get_frame_index(frame_a), get_frame_index(frame_b));
}

Eigen::Matrix3Xd RobotWrapper::com_jacobian()
{
  return model::com_jacobian(model, *data, state.q);
}

Eigen::Matrix3Xd RobotWrapper::com_jacobian_time_variation()
{
  return model::com_jacobian_time_variation(model, *data, state.q, state.qd);
}

Eigen::MatrixXd RobotWrapper::centroidal_map()
{
  return model::centroidal_map(model, *data, state.q);
}

Eigen::VectorXd RobotWrapper::generalized_gravity()
{
  model::generalized_gravity(model, *data, state.q);

  return data->g;
}

Eigen::VectorXd RobotWrapper::non_linear_effects()
{
  model::non_linear_effects(model, *data, state.q, state.qd);

  return data->nle;
}

void RobotWrapper::set_rotor_inertia(const std::string& joint, double inertia)
{
  model.rotor_inertia[get_joint_v_offset(joint)] = inertia;
}

void RobotWrapper::set_gear_ratio(const std::string& joint, double ratio)
{
  model.rotor_gear_ratio[get_joint_v_offset(joint)] = ratio;
}

Eigen::MatrixXd RobotWrapper::mass_matrix()
{
  crba(model, *data, state.q);
  Eigen::MatrixXd M = data->M;

  // We account for inertia by adding the rotor inertia times the squared gear ratio to
  // the diagonal (see Featherstone, Rigid Body Dynamics Algorithm, 2008, end of chapter 9.6)
  for (int k = 0; k < M.rows(); k++)
  {
    M(k, k) += model.rotor_gear_ratio[k] * model.rotor_gear_ratio[k] * model.rotor_inertia[k];
  }

  return M;
}

void RobotWrapper::set_gravity(Eigen::Vector3d gravity)
{
  model.gravity = gravity;
}

void RobotWrapper::integrate(double dt)
{
  // If qd is not set, initialize to 0
  if (state.qd.rows() == 0)
  {
    state.qd = Eigen::VectorXd::Zero(model.nv);
  }

  if (state.qdd.rows() != 0)
  {
    // Integrate acceleration
    state.qd = state.qd + dt * state.qdd;
  }

  // Integrate velocity
  state.q = model.integrate(state.q, state.qd * dt);
}

Eigen::VectorXd RobotWrapper::static_gravity_compensation_torques(RobotWrapper::FrameIndex frameIndex)
{
  auto g = generalized_gravity();
  Eigen::MatrixXd J = frame_jacobian(frameIndex);

  // Jacobian for unactuated and actuated degrees of freedom
  Eigen::MatrixXd Ju = J.block(0, 0, 6, 6);

  // We know that no torque can be provided by floating base. We can then compute f_ext using the unactuated part of
  // the jacobian matrix
  auto f_ext = Ju.transpose().inverse() * g.block(0, 0, 6, 1);

  // Compute the torques
  return g - J.transpose() * f_ext;
}

Eigen::VectorXd RobotWrapper::static_gravity_compensation_torques(std::string frame)
{
  return static_gravity_compensation_torques(get_frame_index(frame));
}

Eigen::VectorXd RobotWrapper::torques_from_acceleration_with_fixed_frame(Eigen::VectorXd qdd_a,
                                                                         RobotWrapper::FrameIndex frameIndex)
{
  auto h = non_linear_effects();
  auto M = mass_matrix();

  // Compute f the forces fixing the frame
  Eigen::MatrixXd J = frame_jacobian(frameIndex);
  Eigen::VectorXd f = J.transpose().block(0, 0, 6, 6).inverse() * h.block(0, 0, 6, 1);

  auto h_a = h.bottomRows(model.nv - 6);
  auto M_a = M.bottomRightCorner(model.nv - 6, model.nv - 6);

  // Compute the torques
  return M_a * qdd_a + h_a - (J.transpose() * f).bottomRows(model.nv - 6);
}

Eigen::VectorXd RobotWrapper::torques_from_acceleration_with_fixed_frame(Eigen::VectorXd qdd_a, std::string frame)
{
  return torques_from_acceleration_with_fixed_frame(qdd_a, get_frame_index(frame));
}

std::vector<std::string> RobotWrapper::joint_names(bool include_floating_base)
{
  std::vector<std::string> joints = model.names;
  if (!include_floating_base)
  {
    joints.erase(std::remove(joints.begin(), joints.end(), "universe"), joints.end());
    joints.erase(std::remove(joints.begin(), joints.end(), "root_joint"), joints.end());
  }
  return joints;
}

std::vector<std::string> RobotWrapper::frame_names()
{
  std::vector<std::string> result;
  for (auto& frame : model.frames)
  {
    result.push_back(frame.name);
  }
  return result;
}

double RobotWrapper::total_mass()
{
  return model.total_mass();
}

void RobotWrapper::add_q_noise(double noise)
{
  auto q_random = model.random_configuration();

  // Adding some noise in direction of a random configuration (except floating base)
  for (int j = 1; j < model.njoints; j++)
  {
    const auto& joint = model.joints[j];
    int k = joint.idx_q;
    if (is_continuous(joint))
    {
      // The noise is added to the angle, so that the configuration stays on the unit circle
      double angle = atan2(state.q(k + 1), state.q(k));
      double random_angle = atan2(q_random(k + 1), q_random(k));
      double delta = atan2(sin(random_angle - angle), cos(random_angle - angle));
      angle += delta * noise;
      state.q(k) = cos(angle);
      state.q(k + 1) = sin(angle);
      continue;
    }
    if (joint.nq != joint.nv || k < 7)
    {
      // Floating base and other joints whose configuration isn't one coordinate per degree of freedom
      continue;
    }
    for (int i = k; i < k + joint.nq; i++)
    {
      if (model.lower_position_limit(i) == std::numeric_limits<double>::lowest() ||
          model.upper_position_limit(i) == std::numeric_limits<double>::max())
      {
        continue;
      }

      state.q(i) += (q_random(i) - state.q(i)) * noise;
    }
  }
}

std::vector<Eigen::Affine3d> RobotWrapper::visual_placements(const Eigen::VectorXd& q)
{
  Data placements_data(model);
  forward_kinematics(model, placements_data, q);
  return visual_model.placements(placements_data);
}

std::vector<Eigen::Affine3d> RobotWrapper::collision_placements(const Eigen::VectorXd& q)
{
  Data placements_data(model);
  forward_kinematics(model, placements_data, q);
  return collision_model.placements(placements_data);
}
}  // namespace placo::model
