#include "placo/model/robot_wrapper.h"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/collision/collision.hpp"
#include "pinocchio/collision/distance.hpp"
#include "placo/tools/utils.h"
#include <boost/filesystem.hpp>
#include <json/json.h>
#include <filesystem>
#include <algorithm>

namespace fs = boost::filesystem;

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
    urdf_filename = model_directory + "/robot.urdf";
  }

  if (urdf_content != "")
  {
    pinocchio::urdf::buildModelFromXML(urdf_content, root_joint, model);
    std::istringstream stream(urdf_content);
    pinocchio::urdf::buildGeom(model, stream, pinocchio::COLLISION, collision_model, model_directory);
  }
  else
  {
    pinocchio::urdf::buildModel(urdf_filename, root_joint, model);
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, collision_model, model_directory);
  }

  if (flags & COLLISION_AS_VISUAL)
  {
    if (urdf_content != "")
    {
      std::istringstream stream(urdf_content);
      pinocchio::urdf::buildGeom(model, stream, pinocchio::COLLISION, visual_model, model_directory);
    }
    else
    {
      pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, visual_model, model_directory);
    }
  }
  else
  {
    if (urdf_content != "")
    {
      std::istringstream stream(urdf_content);
      pinocchio::urdf::buildGeom(model, stream, pinocchio::VISUAL, visual_model, model_directory);
    }
    else
    {
      pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::VISUAL, visual_model, model_directory);
    }
  }

  // Load collisions pairs
  if (!(flags & IGNORE_COLLISIONS))
  {
    if (tools::file_exists(model_directory + "/collisions.json"))
    {
      load_collision_pairs(model_directory + "/collisions.json");
    }
    else
    {
      collision_model.addAllCollisionPairs();
    }
  }

  // Creating data
  data = new pinocchio::Data(model);

  // Assuming that motors with limits both equals to zero are not defined in the
  // URDF, setting them to the maximum possible value
  for (int k = 0; k < model.nq; k++)
  {
    if (model.lowerPositionLimit[k] == 0 && model.upperPositionLimit[k] == 0)
    {
      model.lowerPositionLimit[k] = std::numeric_limits<double>::lowest();
      model.upperPositionLimit[k] = std::numeric_limits<double>::max();
    }
  }

  reset();
  pinocchio::computeAllTerms(model, *data, state.q, state.qd);
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

pinocchio::FrameIndex RobotWrapper::get_frame_index(const std::string& frame)
{
  if (!model.existFrame(frame))
  {
    std::ostringstream oss;
    oss << "Frame with name " << frame << " not found in model";
    throw std::runtime_error(oss.str());
  }

  return model.getFrameId(frame);
}

void RobotWrapper::set_joint(const std::string& name, double value)
{
  state.q[get_joint_offset(name)] = value;
}

double RobotWrapper::get_joint(const std::string& name)
{
  return state.q[get_joint_offset(name)];
}

int RobotWrapper::get_joint_offset(const std::string& name)
{
  if (!model.existJointName(name))
  {
    std::ostringstream oss;
    oss << "Joint with name " << name << " not found in model";
    throw std::runtime_error(oss.str());
  }

  return 7 + model.getJointId(name) - 2;
}

int RobotWrapper::get_joint_v_offset(const std::string& name)
{
  return 6 + model.getJointId(name) - 2;
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
  model.velocityLimit[get_joint_v_offset(name)] = limit;
}

void RobotWrapper::set_torque_limit(const std::string& name, double limit)
{
  model.effortLimit[get_joint_v_offset(name)] = limit;
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
  int k = get_joint_offset(name);
  model.lowerPositionLimit[k] = lower;
  model.upperPositionLimit[k] = upper;
}

std::pair<double, double> RobotWrapper::get_joint_limits(const std::string& name)
{
  int k = get_joint_offset(name);
  return std::make_pair(model.lowerPositionLimit[k], model.upperPositionLimit[k]);
}

Eigen::Affine3d RobotWrapper::get_T_world_fbase()
{
  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();

  transformation.translation() = Eigen::Vector3d(state.q[0], state.q[1], state.q[2]);

  auto rotation = Eigen::Quaterniond(state.q[6], state.q[3], state.q[4], state.q[5]);
  transformation.linear() = rotation.toRotationMatrix();

  return transformation;
}

void RobotWrapper::set_T_world_fbase(Eigen::Affine3d transformation)
{
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
  pinocchio::centerOfMass(model, *data);

  return data->com[0];
}

void RobotWrapper::update_kinematics()
{
  pinocchio::framesForwardKinematics(model, *data, state.q);
  pinocchio::computeJointJacobians(model, *data, state.q);
  pinocchio::computeJointJacobiansTimeVariation(model, *data, state.q, state.qd);
  pinocchio::updateFramePlacements(model, *data);
}

void RobotWrapper::compute_hessians()
{
  pinocchio::computeJointKinematicHessians(model, *data);
}

Eigen::MatrixXd RobotWrapper::get_frame_hessian(FrameIndex frame, int joint_v_index)
{
  // Frame parent joint index
  pinocchio::JointIndex joint_id = model.frames[frame].parent;
  pinocchio::SE3 T = model.frames[frame].placement;

  pinocchio::Tensor<double, 3, 0> H = pinocchio::getJointKinematicHessian(model, *data, joint_id, pinocchio::LOCAL);
  const Eigen::DenseIndex matrix_offset = 6 * model.nv;

  Eigen::Map<Eigen::MatrixXd> hessian(H.data() + joint_v_index * matrix_offset, 6, model.nv);

  return T.inverse().toActionMatrix() * hessian;
}

RobotWrapper::State RobotWrapper::neutral_state()
{
  State state;
  state.q = pinocchio::neutral(model);
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
  for (size_t k = 0; k < collision_model.geometryObjects.size(); k++)
  {
    if (collision_model.geometryObjects[k].parentFrame != std::numeric_limits<FrameIndex>::max())
    {
      std::string name = model.frames[collision_model.geometryObjects[k].parentFrame].name;
      name_to_objects[name].push_back(k);
    }
  }

  collision_model.removeAllCollisionPairs();

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

        collision_model.addCollisionPair(pinocchio::CollisionPair(k1, k2));
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
            collision_model.addCollisionPair(pinocchio::CollisionPair(k1, k2));
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

Eigen::Affine3d RobotWrapper::get_T_world_frame(pinocchio::FrameIndex index)
{
  return tools::pin_se3_to_eigen(data->oMf[index]);
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

void RobotWrapper::set_T_world_frame(pinocchio::FrameIndex frame, Eigen::Affine3d T_world_frameTarget)
{
  Eigen::Affine3d T_world_fbase = get_T_world_fbase();
  Eigen::Affine3d T_world_frame = get_T_world_frame(frame);
  Eigen::Affine3d T_frame_fbase = T_world_frame.inverse() * T_world_fbase;

  set_T_world_fbase(T_world_frameTarget * T_frame_fbase);
}

std::vector<RobotWrapper::Collision> RobotWrapper::self_collisions(bool stop_at_first)
{
  std::vector<Collision> collisions;
  pinocchio::GeometryData geom_data(collision_model);

  // And test all the collision pairs
  pinocchio::computeCollisions(model, *data, collision_model, geom_data, state.q);

  // Print the status of all the collision pairs
  for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k)
  {
    const pinocchio::CollisionPair& cp = collision_model.collisionPairs[k];
    const coal::CollisionResult& cr = geom_data.collisionResults[k];

    if (cr.isCollision())
    {
      Collision collision;

      collision.objA = cp.first;
      collision.objB = cp.second;
      collision.bodyA = collision_model.geometryObjects[cp.first].name;
      collision.parentA = collision_model.geometryObjects[cp.first].parentJoint;
      collision.bodyB = collision_model.geometryObjects[cp.second].name;
      collision.parentB = collision_model.geometryObjects[cp.second].parentJoint;

      for (size_t k = 0; k < cr.numContacts(); k++)
      {
        collision.contacts.push_back(Eigen::Vector3d(cr.getContact(k).pos));
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

std::vector<RobotWrapper::Distance> RobotWrapper::distances()
{
  std::vector<Distance> distances;
  pinocchio::GeometryData geom_data(collision_model);

  // And test all the collision pairs
  pinocchio::computeDistances(model, *data, collision_model, geom_data, state.q);

  for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k)
  {
    const pinocchio::CollisionPair& cp = collision_model.collisionPairs[k];
    const coal::DistanceResult& dr = geom_data.distanceResults[k];

    Distance distance;
    distance.objA = cp.first;
    distance.objB = cp.second;
    distance.min_distance = dr.min_distance;
    distance.pointA = dr.nearest_points[0];
    distance.pointB = dr.nearest_points[1];
    distance.parentA = collision_model.geometryObjects[cp.first].parentJoint;
    distance.parentB = collision_model.geometryObjects[cp.second].parentJoint;
    distances.push_back(distance);
  }

  return distances;
}

static pinocchio::ReferenceFrame string_to_reference(const std::string& reference)
{
  if (reference == "local_world_aligned")
  {
    return pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  }
  else if (reference == "local")
  {
    return pinocchio::ReferenceFrame::LOCAL;
  }
  else if (reference == "world")
  {
    return pinocchio::ReferenceFrame::WORLD;
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

Eigen::MatrixXd RobotWrapper::frame_jacobian(pinocchio::FrameIndex frame, pinocchio::ReferenceFrame ref)
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(6, model.nv);
  jacobian.setZero();
  pinocchio::getFrameJacobian(model, *data, frame, ref, jacobian);

  return jacobian;
}

Eigen::MatrixXd RobotWrapper::frame_jacobian_time_variation(const std::string& frame, const std::string& reference)
{
  return frame_jacobian_time_variation(get_frame_index(frame), string_to_reference(reference));
}

Eigen::MatrixXd RobotWrapper::frame_jacobian_time_variation(pinocchio::FrameIndex frame, pinocchio::ReferenceFrame ref)
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(6, model.nv);
  jacobian.setZero();

  pinocchio::getFrameJacobianTimeVariation(model, *data, frame, ref, jacobian);

  return jacobian;
}

Eigen::MatrixXd RobotWrapper::joint_jacobian(const std::string& joint, const std::string& reference)
{
  return joint_jacobian(model.getJointId(joint), string_to_reference(reference));
}

Eigen::MatrixXd RobotWrapper::joint_jacobian(pinocchio::JointIndex joint, pinocchio::ReferenceFrame ref)
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(6, model.nv);
  jacobian.setZero();
  pinocchio::getJointJacobian(model, *data, joint, ref, jacobian);

  return jacobian;
}

Eigen::MatrixXd RobotWrapper::joint_jacobian_time_variation(const std::string& joint, const std::string& reference)
{
  return joint_jacobian(model.getJointId(joint), string_to_reference(reference));
}

Eigen::MatrixXd RobotWrapper::joint_jacobian_time_variation(pinocchio::JointIndex joint, pinocchio::ReferenceFrame ref)
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> jacobian(6, model.nv);
  jacobian.setZero();
  pinocchio::getJointJacobianTimeVariation(model, *data, joint, ref, jacobian);

  return jacobian;
}

Eigen::MatrixXd RobotWrapper::relative_position_jacobian(pinocchio::FrameIndex frame_a, pinocchio::FrameIndex frame_b)
{
  auto T_world_a = get_T_world_frame(frame_a);
  auto T_world_b = get_T_world_frame(frame_b);
  auto T_a_b = T_world_a.inverse() * T_world_b;

  Eigen::MatrixXd R_world_a = T_world_a.linear();

  Eigen::MatrixXd J_a_pos = frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED).block(0, 0, 3, model.nv);
  Eigen::MatrixXd J_a_rot = frame_jacobian(frame_a, pinocchio::LOCAL_WORLD_ALIGNED).block(3, 0, 3, model.nv);
  Eigen::MatrixXd J_b_pos = frame_jacobian(frame_b, pinocchio::LOCAL_WORLD_ALIGNED).block(0, 0, 3, model.nv);

  return (R_world_a.transpose() * (J_b_pos - J_a_pos) +
          pinocchio::skew(T_a_b.translation()) * R_world_a.transpose() * J_a_rot);
}

Eigen::MatrixXd RobotWrapper::relative_position_jacobian(const std::string& frame_a, const std::string& frame_b)
{
  return relative_position_jacobian(get_frame_index(frame_a), get_frame_index(frame_b));
}

Eigen::Matrix3Xd RobotWrapper::com_jacobian()
{
  return pinocchio::jacobianCenterOfMass(model, *data, state.q);
}

Eigen::Matrix3Xd RobotWrapper::com_jacobian_time_variation()
{
  // See https://github.com/stack-of-tasks/pinocchio/issues/1297
  return pinocchio::computeCentroidalMapTimeVariation(model, *data, state.q, state.qd).topRows(3) / total_mass();
}

Eigen::MatrixXd RobotWrapper::centroidal_map()
{
  return pinocchio::computeCentroidalMap(model, *data, state.q);
}

Eigen::VectorXd RobotWrapper::generalized_gravity()
{
  pinocchio::computeGeneralizedGravity(model, *data, state.q);

  return data->g;
}

Eigen::VectorXd RobotWrapper::non_linear_effects()
{
  return pinocchio::nonLinearEffects(model, *data, state.q, state.qd);
}

void RobotWrapper::set_rotor_inertia(const std::string& joint, double inertia)
{
  model.rotorInertia[get_joint_v_offset(joint)] = inertia;
}

void RobotWrapper::set_gear_ratio(const std::string& joint, double ratio)
{
  model.rotorGearRatio[get_joint_v_offset(joint)] = ratio;
}

Eigen::MatrixXd RobotWrapper::mass_matrix()
{
  pinocchio::crba(model, *data, state.q);
  data->M.triangularView<Eigen::StrictlyLower>() = data->M.transpose().triangularView<Eigen::StrictlyLower>();
  Eigen::MatrixXd M = data->M;

  // We account for inertia by adding the rotor inertia times the squared gear ratio to
  // the diagonal (see Featherstone, Rigid Body Dynamics Algorithm, 2008, end of chapter 9.6)
  for (int k = 0; k < M.rows(); k++)
  {
    M(k, k) += model.rotorGearRatio[k] * model.rotorGearRatio[k] * model.rotorInertia[k];
  }

  return M;
}

void RobotWrapper::set_gravity(Eigen::Vector3d gravity)
{
  model.gravity.linear() = gravity;
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
  state.q = pinocchio::integrate(model, state.q, state.qd * dt);
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
  double mass = 0.0;

  for (auto& body : model.inertias)
  {
    mass += body.mass();
  }

  return mass;
}

void RobotWrapper::add_q_noise(double noise)
{
  auto q_random = pinocchio::randomConfiguration(model);

  // Adding some noise in direction of a random configuration (except floating base)
  for (int k = 7; k < model.nq; k++)
  {
    if (model.lowerPositionLimit(k) == std::numeric_limits<double>::lowest() ||
        model.upperPositionLimit(k) == std::numeric_limits<double>::max())
    {
      continue;
    }

    state.q(k) += (q_random(k) - state.q(k)) * noise;
  }
}
}  // namespace placo::model
