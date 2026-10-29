#pragma once

#include <vector>
#include <Eigen/Dense>
#include "placo/model/spatial.h"

namespace placo::model
{
class Model;

/**
 * @brief Quantities computed by the rigid body algorithms (see algorithms.h) for a model
 */
struct Data
{
  Data(const Model& model);

  /**
   * @brief Placement of each joint in its parent joint frame
   */
  std::vector<Eigen::Affine3d> liMi;

  /**
   * @brief Placement of each joint in the world
   */
  std::vector<Eigen::Affine3d> oMi;

  /**
   * @brief Placement of each frame in the world
   */
  std::vector<Eigen::Affine3d> oMf;

  /**
   * @brief Spatial velocity of each joint, expressed in the joint frame
   */
  std::vector<Vector6d> v;

  /**
   * @brief Spatial velocity of each joint, expressed in the world frame
   */
  std::vector<Vector6d> ov;

  /**
   * @brief Joint Jacobians: column k is the spatial velocity (world frame) of the joint of the degree of freedom k
   * for a unit velocity of this degree of freedom
   */
  Matrix6Xd J;

  /**
   * @brief Time variation of J
   */
  Matrix6Xd dJ;

  /**
   * @brief Mass matrix
   */
  Eigen::MatrixXd M;

  /**
   * @brief Non-linear effects (Coriolis, centrifugal and gravity)
   */
  Eigen::VectorXd nle;

  /**
   * @brief Generalized gravity
   */
  Eigen::VectorXd g;

  /**
   * @brief Center of mass of the robot, in the world
   */
  Eigen::Vector3d com;

  /**
   * @brief Mass of the subtree of each joint
   */
  std::vector<double> subtree_mass;

  /**
   * @brief Center of mass of the subtree of each joint, in the world
   */
  std::vector<Eigen::Vector3d> subtree_com;

  /**
   * @brief Linear momentum of the subtree of each joint, in the world
   */
  std::vector<Eigen::Vector3d> subtree_momentum;

  /**
   * @brief Composite rigid body inertia of the subtree of each joint, in the world frame
   */
  std::vector<Matrix6d> oYcrb;

  /**
   * @brief Spatial accelerations and forces of the joints (inverse dynamics)
   */
  std::vector<Vector6d> a, f;
};
}  // namespace placo::model
