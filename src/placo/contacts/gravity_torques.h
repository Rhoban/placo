#pragma once

#include <Eigen/Dense>
#include "placo/model/robot_wrapper.h"

namespace placo
{
struct GravityTorques
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

  struct Contact
  {
    std::string frame_name;

    enum Type
    {
      Planar = 0,
      Point = 1
    };

    Type type;

    // For planar contacts, the length and width of the contact rectangle
    double length;
    double width;

    // Friction coefficient
    double mu = 1.;

    Contact(const std::string& frame_name, Type type, double mu = 1., double length = 0.0, double width = 0.0);
  };

  // Contacts
  std::vector<Contact> contacts;

  // Passive joints
  std::set<std::string> passive_joints;

  void add_contact(Contact contact);

  void set_passive(const std::string& joint_name, bool is_passive = true);

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
   * @param contacts list of frames which are unitaleral contacts
   * @param contact_length contact rectangles length (you might consider some margin)
   * @param contact_width contact rectangles width (you might consider some margin)
   * @param mu friction coefficient
   * @return
   */
  Result compute_gravity_torques(RobotWrapper& robot);
};
}  // namespace placo