#pragma once

#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"

namespace placo
{
class GravityTorques
{
public:
  struct Result
  {
    // Checks if the gravity computation is a success
    bool success;

    // Torques computed by the solver
    Eigen::VectorXd tau;

    // Contact forces (expressed in body), each row is a 6 dimensionnal wrench
    // appearing in the same order as contacts during solving.
    Eigen::MatrixXd contact_wrenches;
  };

  /**
   * @brief Computes the torques required to compensate gravity given a set of unilateral contacts. This
   * formulates and try to solve a QP problem with the following properties:
   *
   * - Objective function:
   *   - Trying to minimize the moments at contact (as a result, ZMP is tried to be kept as much as
   *     possible at the center of the contact)
   *   - Trying to minimize the required torques
   * - Constraints:
   *   - Equation of motion: tau + sum(J^T f) = g
   *   - The contact fz are positive (contacts are unilaterals)
   *   - The ZMP is kept in the admissible rectangles (using foot_length and foot_width)
   *   - Friction cones using the given mu ratio
   *
   * (In the future, this API might change in favour of more versatile contacts representation)
   *
   * @param robot robot wrapper
   * @param unilateral_contacts list of frames which are unitaleral contacts
   * @param contact_length contact rectangles length (you might consider some margin)
   * @param contact_width contact rectangles width (you might consider some margin)
   * @param mu friction coefficient
   * @return
   */
  static Result compute_gravity_torques(RobotWrapper& robot, std::vector<std::string> unilateral_contacts,
                                        double contact_length, double contact_width, double mu = 1.);
};
}  // namespace placo