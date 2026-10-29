#pragma once

#include <vector>
#include <Eigen/Dense>
#include "placo/model/model.h"
#include "placo/model/data.h"

namespace placo::model
{
/**
 * @brief Computes the placements of the joints (data.liMi and data.oMi)
 */
void forward_kinematics(const Model& model, Data& data, const Eigen::VectorXd& q);

/**
 * @brief Computes the placements of the joints and the joint Jacobians (data.J)
 */
void compute_joint_jacobians(const Model& model, Data& data, const Eigen::VectorXd& q);

/**
 * @brief Computes the placements of the joints, their velocities (data.v, data.ov), the joint Jacobians and their
 * time variation (data.J and data.dJ)
 */
void compute_joint_jacobians_time_variation(const Model& model, Data& data, const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& qd);

/**
 * @brief Updates the placements of the frames (data.oMf) from the placements of the joints
 */
void update_frame_placements(const Model& model, Data& data);

/**
 * @brief Jacobian of a point attached to a joint, computed from the joint Jacobians (see compute_joint_jacobians),
 * for the given columns (degrees of freedom), which are sorted
 * @param model the model
 * @param data the data
 * @param joint the joint
 * @param support the degrees of freedom of the joint support (sorted)
 * @param T_world_point placement of the point (frame) in the world
 * @param ref reference frame
 * @param columns columns of the Jacobian to compute (sorted)
 * @param J the Jacobian (6 x columns.size())
 * @param time_variation if true, the time variation of the Jacobian is computed instead (requires
 * compute_joint_jacobians_time_variation)
 */
void point_jacobian(const Model& model, const Data& data, int joint, const std::vector<int>& support,
                    const Eigen::Affine3d& T_world_point, ReferenceFrame ref, const std::vector<int>& columns,
                    Eigen::MatrixXd& J, bool time_variation);

/**
 * @brief Computes the mass matrix (data.M), with the composite rigid body algorithm
 */
void crba(const Model& model, Data& data, const Eigen::VectorXd& q);

/**
 * @brief Inverse dynamics (recursive Newton-Euler algorithm): torques needed to have the acceleration qdd
 */
Eigen::VectorXd rnea(const Model& model, Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
                     const Eigen::VectorXd& qdd);

/**
 * @brief Computes the non linear effects (data.nle): Coriolis, centrifugal and gravity torques
 */
void non_linear_effects(const Model& model, Data& data, const Eigen::VectorXd& q, const Eigen::VectorXd& qd);

/**
 * @brief Computes the generalized gravity (data.g)
 */
void generalized_gravity(const Model& model, Data& data, const Eigen::VectorXd& q);

/**
 * @brief Computes the center of mass (data.com), and the subtrees masses and centers of mass, from the placements
 * of the joints (data.oMi)
 */
void center_of_mass(const Model& model, Data& data);

/**
 * @brief Jacobian of the center of mass (3 x nv)
 */
Eigen::Matrix3Xd com_jacobian(const Model& model, Data& data, const Eigen::VectorXd& q);

/**
 * @brief Time variation of the Jacobian of the center of mass (3 x nv)
 */
Eigen::Matrix3Xd com_jacobian_time_variation(const Model& model, Data& data, const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& qd);

/**
 * @brief Centroidal momentum matrix (6 x nv): maps the velocity to the momentum of the robot, expressed at its
 * center of mass (world axes), as [linear; angular]
 */
Eigen::MatrixXd centroidal_map(const Model& model, Data& data, const Eigen::VectorXd& q);

/**
 * @brief Kinematic Hessian of a frame: derivative of the frame Jacobian (local reference frame) with respect to the
 * degree of freedom dof, from the joint Jacobians (see compute_joint_jacobians)
 */
Eigen::MatrixXd frame_hessian(const Model& model, const Data& data, int frame, int dof);
}  // namespace placo::model
