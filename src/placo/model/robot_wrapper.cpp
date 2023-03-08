#include "placo/model/robot_wrapper.h"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "placo/utils.h"
#include "rhoban_utils/util.h"
#include <jsoncpp/json/json.h>
#include <filesystem>

namespace placo
{
RobotWrapper::RobotWrapper(std::string model_directory) : model_directory(model_directory)
{
}

bool RobotWrapper::Collision::operator==(const Collision& other)
{
  return (objA == other.objA && objB == other.objB);
}

bool RobotWrapper::Distance::operator==(const Distance& other)
{
  return (objA == other.objA && objB == other.objB);
}

void RobotWrapper::load()
{
  std::string urdf_filename = model_directory + "/robot.urdf";
  pinocchio::urdf::buildModel(urdf_filename, root_joint, model);
  pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, collision_model, model_directory);
  pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::VISUAL, visual_model, model_directory);

  // Load collisions pairs
  if (rhoban_utils::file_exists(model_directory + "/collisions.json"))
  {
    load_collisions_pairs(model_directory + "/collisions.json");
  }
  else
  {
    collision_model.addAllCollisionPairs();
  }

  // Creating data
  data = new pinocchio::Data(model);

  // Ensuring expected DOFs are present
  auto _expected_dofs = expected_dofs();
  for (auto& dof : expected_dofs())
  {
    get_joint_offset(dof);
  }

  if (_expected_dofs.size() > 0 && _expected_dofs.size() + 7 != model.nq)
  {
    std::ostringstream oss;
    oss << "Found " << model.nq << " DOFs, expected " << (_expected_dofs.size() + 7) << std::endl;
    throw std::runtime_error(oss.str());
  }

  // Ensuring expected frames are present
  for (auto& frame : expected_frames())
  {
    get_frame_index(frame);
  }

  // Assuming that motors with limits both equals to zero are not defined in the
  // URDF, setting them to PI
  for (int k = 0; k < model.nq; k++)
  {
    if (model.lowerPositionLimit[k] == 0 && model.upperPositionLimit[k] == 0)
    {
      model.lowerPositionLimit[k] = -M_PI;
      model.upperPositionLimit[k] = M_PI;
    }
  }

  reset();
  pinocchio::computeAllTerms(model, *data, state.q, state.qd);
  update_kinematics();

  if (self_collisions().size() > 0)
  {
    throw std::runtime_error("Robot is self colliding in neutral position");
  }
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
}

RobotWrapper::State RobotWrapper::neutral_state()
{
  State state;
  state.q = pinocchio::neutral(model);
  state.qd = Eigen::VectorXd(model.nv);
  state.qd.setZero();

  return state;
}

void RobotWrapper::load_collisions_pairs(const std::string& filename)
{
  // Reading collision pairs
  Json::Value collisions;

  collision_model.removeAllCollisionPairs();

  std::ifstream f(filename);
  if (!f.is_open())
  {
    throw std::runtime_error("Can't load collision pairs");
  }
  f >> collisions;

  for (int k = 0; k < collisions.size(); k++)
  {
    Json::Value& entry = collisions[k];
    if (entry.size() == 2)
    {
      int pair1 = entry[0].asInt();
      int pair2 = entry[1].asInt();

      collision_model.addCollisionPair(pinocchio::CollisionPair(pair1, pair2));
    }
  }
}

Eigen::Affine3d RobotWrapper::get_T_world_frame(const std::string& frame)
{
  return get_T_world_frame(get_frame_index(frame));
}

Eigen::Affine3d RobotWrapper::get_T_world_frame(pinocchio::FrameIndex index)
{
  return pin_se3_to_eigen(data->oMf[index]);
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
    const hpp::fcl::CollisionResult& cr = geom_data.collisionResults[k];

    if (cr.isCollision())
    {
      Collision collision;

      collision.objA = cp.first;
      collision.objB = cp.second;
      collision.bodyA = collision_model.geometryObjects[cp.first].name;
      collision.parentA = collision_model.geometryObjects[cp.first].parentJoint;
      collision.bodyB = collision_model.geometryObjects[cp.second].name;
      collision.parentB = collision_model.geometryObjects[cp.second].parentJoint;

      for (int k = 0; k < cr.numContacts(); k++)
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
    const hpp::fcl::DistanceResult& dr = geom_data.distanceResults[k];

    Distance distance;
    distance.objA = cp.first;
    distance.objB = cp.second;
    distance.min_distance = dr.min_distance;
    distance.pointA = dr.nearest_points[0];
    distance.pointB = dr.nearest_points[1];
    distance.parentA = collision_model.geometryObjects[cp.first].parentJoint;
    distance.parentB = collision_model.geometryObjects[cp.second].parentJoint;
    distance.normal = dr.normal;
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
  pinocchio::computeFrameJacobian(model, *data, state.q, frame, ref, jacobian);

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
  pinocchio::computeJointJacobian(model, *data, state.q, joint, jacobian);
  pinocchio::getJointJacobian(model, *data, joint, ref, jacobian);

  return jacobian;
}

Eigen::Matrix3Xd RobotWrapper::com_jacobian()
{
  return pinocchio::jacobianCenterOfMass(model, *data, state.q);
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

Eigen::VectorXd RobotWrapper::static_gravity_compensation_torques(RobotWrapper::FrameIndex frameIndex)
{
  auto g = generalized_gravity();
  auto J = frame_jacobian(frameIndex);

  // Jacobian for unactuated and actuated degrees of freedom
  auto Ju = J.block(0, 0, 6, 6);

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

std::vector<std::string> RobotWrapper::joint_names()
{
  return model.names;
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

std::vector<std::string> RobotWrapper::expected_dofs()
{
  return {};
}

std::vector<std::string> RobotWrapper::expected_frames()
{
  return {};
}
}  // namespace placo
