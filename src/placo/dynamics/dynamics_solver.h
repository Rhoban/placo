#pragma once

#include <Eigen/Dense>
#include <set>
#include "placo/model/robot_wrapper.h"
#include "placo/problem/problem.h"
#include "placo/control/axises_mask.h"
#include "placo/dynamics/task.h"
#include "placo/dynamics/position_task.h"
#include "placo/dynamics/orientation_task.h"

namespace placo
{
namespace dynamics
{
class DynamicsSolver
{
public:
  struct Result
  {
    // Checks if the gravity computation is a success
    bool success;

    // Torques computed by the solver
    Eigen::VectorXd tau;

    // Accelerations computed by the solver
    Eigen::VectorXd qdd;
  };

  struct Contact
  {
    struct Wrench
    {
      Eigen::MatrixXd J;
      Expression f;
    };

    std::string frame_name = "";

    enum Type
    {
      Fixed = 0,
      Planar = 1,
      Point = 2
    };

    Type type = Point;

    // Configures the contact
    void configure(const std::string& frame_name, Type type, double mu = 1., double length = 0., double width = 0.);

    // For planar contacts, the length and width of the contact rectangle
    // Length is along x axis in local frame, and width along y axis
    double length = 0.;
    double width = 0.;

    // Friction coefficient
    double mu = 1.;

    // Weights for optimization
    double weight_forces = 1e-6;
    double weight_moments = 1e-3;

    // Adds the wrench to the problem
    Wrench add_wrench(RobotWrapper& robot, Problem& problem);

    // Returns the ZMP of the contact
    Eigen::Vector3d zmp();

    // Wrench computed by the solver
    Eigen::MatrixXd wrench;
    Variable* variable;
  };

  struct LoopClosure
  {
    std::string frame_a;
    std::string frame_b;
    AxisesMask mask;
  };

  DynamicsSolver(RobotWrapper& robot);
  virtual ~DynamicsSolver();

  // Contacts
  std::vector<Contact*> contacts;

  // Passive joints
  std::set<std::string> passive_joints;

  // Desired accelerations
  Eigen::VectorXd qdd_desired = Eigen::VectorXd();

  /**
   * @brief Adds a contact to the solver
   * @param contact the contact to add
   */
  Contact& add_contact();

  /**
   * @brief Sets a DoF as passive
   * @param joint_name the DoF name
   * @param is_passive true if the DoF should be passive
   */
  void set_passive(const std::string& joint_name, bool is_passive = true);

  /**
   * @brief Adds a loop closing constraint (xy should be zero)
   * @param frame_a
   * @param frame_b
   */
  void add_loop_closing_constraint(const std::string& frame_a, const std::string& frame_b, const std::string& axises);

  PositionTask& add_position_task(pinocchio::FrameIndex frame_index, Eigen::Vector3d target_world);
  PositionTask& add_position_task(std::string frame_name, Eigen::Vector3d target_world);
  OrientationTask& add_orientation_task(pinocchio::FrameIndex frame_index, Eigen::Matrix3d R_world_frame);
  OrientationTask& add_orientation_task(std::string frame_name, Eigen::Matrix3d R_world_frame);

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
  Result solve();

  RobotWrapper& robot;

  void remove_task(Task* task);

  int N;

protected:
  std::vector<LoopClosure> loop_closing_constraints;

  std::set<Task*> tasks;

  // Task id (this is only useful when task names are not specified, each task will have an unique ID)
  int task_id = 0;

  template <typename T>
  T& add_task(T* task)
  {
    task_id += 1;
    task->solver = this;
    std::ostringstream oss;
    oss << "Task_" << task_id;
    task->name = oss.str();
    tasks.insert(task);

    return *task;
  }
};
}  // namespace dynamics
}  // namespace placo