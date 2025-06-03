#include "placo/humanoid/walk_pattern_generator.h"
#include "placo/humanoid/footsteps_planner.h"
#include "placo/problem/polygon_constraint.h"
#include "placo/tools/utils.h"
#include <chrono>

namespace placo::humanoid
{
using namespace placo::problem;
using namespace placo::tools;

WalkPatternGenerator::TrajectoryPart::TrajectoryPart(FootstepsPlanner::Support support, double t_start)
  : t_start(t_start), support(support)
{
}

WalkPatternGenerator::Trajectory::Trajectory() : left_foot_yaw(true), right_foot_yaw(true), trunk_yaw(true)
{
  T.setIdentity();
}

WalkPatternGenerator::Trajectory::Trajectory(double com_target_z, double t_start, double trunk_pitch, double trunk_roll)
  : t_start(t_start)
  , com_target_z(com_target_z)
  , trunk_pitch(trunk_pitch)
  , trunk_roll(trunk_roll)
  , left_foot_yaw(true)
  , right_foot_yaw(true)
  , trunk_yaw(true)
{
  T.setIdentity();
}

WalkPatternGenerator::WalkPatternGenerator(HumanoidRobot& robot, HumanoidParameters& parameters)
  : robot(robot), parameters(parameters)
{
  omega = LIPM::compute_omega(parameters.walk_com_height);
  omega_2 = pow(omega, 2);
}

static Eigen::Affine3d _buildFrame(Eigen::Vector3d position, double orientation)
{
  Eigen::Affine3d frame = Eigen::Affine3d::Identity();

  frame.translation() = position;
  frame.linear() = Eigen::AngleAxisd(orientation, Eigen::Vector3d::UnitZ()).matrix();

  return frame;
}

static WalkPatternGenerator::TrajectoryPart& _findPart(std::vector<WalkPatternGenerator::TrajectoryPart>& parts,
                                                       double t, int* index = nullptr)
{
  if (parts.size() == 0)
  {
    throw std::runtime_error("Can't find a part in a trajectory that has 0 parts");
  }

  int low = 0;
  int high = parts.size() - 1;

  while (low != high)
  {
    int mid = (low + high) / 2;

    WalkPatternGenerator::TrajectoryPart& part = parts[mid];

    if (t < part.t_start)
    {
      high = mid;
    }
    else if (t > part.t_end)
    {
      low = mid + 1;
    }
    else
    {
      if (index != nullptr)
      {
        *index = mid;
      }
      return part;
    }
  }

  if (index != nullptr)
  {
    *index = low;
  }
  return parts[low];
}

bool WalkPatternGenerator::Trajectory::is_flying(HumanoidRobot::Side side, double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  return (!part.support.is_both() && part.support.side() == HumanoidRobot::other_side(side));
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_left(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  if (is_flying(HumanoidRobot::Left, t))
  {
    return T * _buildFrame(part.swing_trajectory.pos(t), left_foot_yaw.pos(t));
  }
  return T * _buildFrame(part.support.footstep_frame(HumanoidRobot::Left).translation(), left_foot_yaw.pos(t));
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_right(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  if (is_flying(HumanoidRobot::Right, t))
  {
    return T * _buildFrame(part.swing_trajectory.pos(t), right_foot_yaw.pos(t));
  }
  return T * _buildFrame(part.support.footstep_frame(HumanoidRobot::Right).translation(), right_foot_yaw.pos(t));
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_foot(HumanoidRobot::Side side, double t)
{
  return (side == HumanoidRobot::Left) ? get_T_world_left(t) : get_T_world_right(t);
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_v_world_left(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  if (part.support.side() == HumanoidRobot::Right)
  {
    return T.linear() * part.swing_trajectory.vel(t);
  }
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_v_world_right(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  if (part.support.side() == HumanoidRobot::Left)
  {
    return T.linear() * part.swing_trajectory.vel(t);
  }
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_v_world_foot(HumanoidRobot::Side side, double t)
{
  return (side == HumanoidRobot::Left) ? get_v_world_left(t) : get_v_world_right(t);
}

double WalkPatternGenerator::Trajectory::get_yaw_world_left(double t)
{
  return frame_yaw(T.rotation()) + left_foot_yaw.pos(t);
}

double WalkPatternGenerator::Trajectory::get_yaw_world_right(double t)
{
  return frame_yaw(T.rotation()) + right_foot_yaw.pos(t);
}

double WalkPatternGenerator::Trajectory::get_yaw_world_foot(HumanoidRobot::Side side, double t)
{
  return (side == HumanoidRobot::Left) ? get_yaw_world_left(t) : get_yaw_world_right(t);
}

double WalkPatternGenerator::Trajectory::get_yaw_world_trunk(double t)
{
  return frame_yaw(T.rotation()) + trunk_yaw.pos(t);
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_p_world_CoM(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  Eigen::Vector3d pos = Eigen::Vector3d(part.com_trajectory.pos(t).x(), part.com_trajectory.pos(t).y(), com_target_z);
  return T * pos;
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_v_world_CoM(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  Eigen::Vector3d vel = Eigen::Vector3d(part.com_trajectory.vel(t).x(), part.com_trajectory.vel(t).y(), 0);
  return T.linear() * vel;
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_a_world_CoM(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  Eigen::Vector3d acc = Eigen::Vector3d(part.com_trajectory.acc(t).x(), part.com_trajectory.acc(t).y(), 0);
  return T.linear() * acc;
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_j_world_CoM(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  Eigen::Vector3d jerk = Eigen::Vector3d(part.com_trajectory.jerk(t).x(), part.com_trajectory.jerk(t).y(), 0);
  return T.linear() * jerk;
}

Eigen::Vector2d WalkPatternGenerator::Trajectory::get_p_world_DCM(double t, double omega)
{
  return get_p_world_CoM(t).head(2) + (1 / omega) * get_v_world_CoM(t).head(2);
}

Eigen::Vector2d WalkPatternGenerator::Trajectory::get_p_world_ZMP(double t, double omega)
{
  return get_p_world_CoM(t).head(2) - (1 / pow(omega, 2)) * get_a_world_CoM(t).head(2);
}

Eigen::Matrix3d WalkPatternGenerator::Trajectory::get_R_world_trunk(double t)
{
  Eigen::Matrix3d R_world_trunk = Eigen::AngleAxisd(trunk_yaw.pos(t), Eigen::Vector3d::UnitZ()).matrix() *
                                  Eigen::AngleAxisd(trunk_pitch, Eigen::Vector3d::UnitY()).matrix() *
                                  Eigen::AngleAxisd(trunk_roll, Eigen::Vector3d::UnitX()).matrix();
  return T.linear() * R_world_trunk;
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_p_support_CoM(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  Eigen::Affine3d T_support_world = placo::tools::flatten_on_floor(get_T_world_foot(part.support.side(), t)).inverse();
  return T_support_world * get_p_world_CoM(t);
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_v_support_CoM(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  Eigen::Affine3d T_support_world = placo::tools::flatten_on_floor(get_T_world_foot(part.support.side(), t)).inverse();
  return T_support_world.linear() * get_v_world_CoM(t);
}

Eigen::Vector2d WalkPatternGenerator::Trajectory::get_p_support_DCM(double t, double omega)
{
  return get_p_support_CoM(t).head(2) + (1 / omega) * get_v_support_CoM(t).head(2);
}

HumanoidRobot::Side WalkPatternGenerator::Trajectory::support_side(double t)
{
  return _findPart(parts, t).support.side();
}

bool WalkPatternGenerator::Trajectory::support_is_both(double t)
{
  return _findPart(parts, t).support.is_both();
}

placo::tools::CubicSpline& WalkPatternGenerator::Trajectory::foot_yaw(HumanoidRobot::Side side)
{
  if (side == HumanoidRobot::Left)
  {
    return left_foot_yaw;
  }
  else
  {
    return right_foot_yaw;
  }
}

FootstepsPlanner::Support WalkPatternGenerator::Trajectory::get_support(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  return T * part.support;
}

int WalkPatternGenerator::Trajectory::remaining_supports(double t)
{
  int index;
  _findPart(parts, t, &index);

  return parts.size() - index - 1;
}

FootstepsPlanner::Support WalkPatternGenerator::Trajectory::get_next_support(double t, int n)
{
  int index;
  _findPart(parts, t, &index);
  if (index + n >= (int)parts.size())
  {
    return T * parts.back().support;
  }
  return T * parts[index + n].support;
}

FootstepsPlanner::Support WalkPatternGenerator::Trajectory::get_prev_support(double t, int n)
{
  TrajectoryPart& part = _findPart(parts, t);
  for (int i = 0; i < n; i++)
  {
    part = _findPart(parts, part.t_start - 1e-4);
  }
  return T * part.support;
}

std::vector<FootstepsPlanner::Support> WalkPatternGenerator::Trajectory::get_supports()
{
  std::vector<FootstepsPlanner::Support> supports;
  for (auto part : parts)
  {
    supports.push_back(T * part.support);
  }
  return supports;
}

void WalkPatternGenerator::Trajectory::apply_transform(Eigen::Affine3d T_)
{
  T = T_ * T;
}

double WalkPatternGenerator::Trajectory::get_part_t_start(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  return part.t_start;
}

double WalkPatternGenerator::Trajectory::get_part_t_end(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  return part.t_end;
}

Eigen::Vector2d WalkPatternGenerator::Trajectory::get_part_end_dcm(double t, double omega)
{
  TrajectoryPart& part = _findPart(parts, t);
  return get_p_world_DCM(part.t_end, omega);
}

void WalkPatternGenerator::Trajectory::add_supports(double t, FootstepsPlanner::Support& support)
{
  for (auto footstep : support.footsteps)
  {
    auto T_world_foot = footstep.frame;
    foot_yaw(footstep.side).add_point(t, frame_yaw(T_world_foot.rotation()), 0);
  }
}

void WalkPatternGenerator::plan_dbl_support(Trajectory& trajectory, int part_index)
{
  TrajectoryPart& part = trajectory.parts[part_index];
  trajectory.add_supports(part.t_end, part.support);
  trajectory.trunk_yaw.add_point(part.t_end, frame_yaw(part.support.frame().rotation()), 0);
}

void WalkPatternGenerator::plan_sgl_support(Trajectory& trajectory, int part_index, Trajectory* old_trajectory)
{
  TrajectoryPart& part = trajectory.parts[part_index];

  // Swing trajectory
  HumanoidRobot::Side flying_side = HumanoidRobot::other_side(part.support.footsteps[0].side);
  Eigen::Affine3d T_world_end = trajectory.parts[part_index + 1].support.footstep_frame(flying_side);

  double virt_duration = support_default_duration(part.support) * part.support.time_ratio;

  if (part_index == 0 && old_trajectory != nullptr)
  {
    Eigen::Vector3d start = old_trajectory->get_T_world_foot(flying_side, trajectory.t_start).translation();
    Eigen::Vector3d start_vel = old_trajectory->get_v_world_foot(flying_side, trajectory.t_start);
    part.swing_trajectory = SwingFootCubic::make_trajectory(
        part.t_start, virt_duration, parameters.walk_foot_height, parameters.walk_foot_rise_ratio, start,
        T_world_end.translation(), part.support.elapsed_ratio, start_vel);

    // Initiate the flying foot yaw spline with the old trajectory foot yaw position and velocity
    trajectory.foot_yaw(flying_side)
        .add_point(trajectory.t_start, old_trajectory->get_yaw_world_foot(flying_side, trajectory.t_start),
                   old_trajectory->foot_yaw(flying_side).vel(trajectory.t_start));
  }
  else
  {
    Eigen::Vector3d start = trajectory.parts[part_index - 1].support.footstep_frame(flying_side).translation();
    part.swing_trajectory =
        SwingFootCubic::make_trajectory(part.t_start, virt_duration, parameters.walk_foot_height,
                                        parameters.walk_foot_rise_ratio, start, T_world_end.translation());
  }

  trajectory.foot_yaw(flying_side).add_point(part.t_end, frame_yaw(T_world_end.rotation()), 0);

  // The trunk orientation follow the steps orientation
  if (!parameters.has_double_support())
  {
    trajectory.trunk_yaw.add_point(part.t_end, frame_yaw(T_world_end.rotation()), 0);
  }

  // Support foot remaining steady
  trajectory.add_supports(part.t_end, part.support);
}

void WalkPatternGenerator::plan_feet_trajectories(Trajectory& trajectory, Trajectory* old_trajectory)
{
  // Add the initial position to the trajectory
  trajectory.add_supports(trajectory.t_start, trajectory.parts[0].support);
  if (old_trajectory == nullptr)
  {
    // Starting the walk in double support, we set the trunk yaw aligned with the first support
    trajectory.trunk_yaw.add_point(trajectory.t_start, frame_yaw(trajectory.parts[0].support.frame().rotation()), 0);
  }
  else
  {
    // Initiate the trunk_yaw spline with the old trajectory trunk_yaw position and velocity
    trajectory.trunk_yaw.add_point(trajectory.t_start, old_trajectory->get_yaw_world_trunk(trajectory.t_start),
                                   old_trajectory->trunk_yaw.vel(trajectory.t_start));
  }

  for (int i = 0; i < (int)trajectory.parts.size(); i++)
  {
    // Single support
    if (trajectory.parts[i].support.footsteps.size() == 1)
    {
      plan_sgl_support(trajectory, i, old_trajectory);
    }
    // Double support
    else
    {
      plan_dbl_support(trajectory, i);
    }
  }
}

void WalkPatternGenerator::constrain_lipm(problem::Problem& problem, LIPM& lipm, FootstepsPlanner::Support& support,
                                          double omega_2, HumanoidParameters& parameters)
{
  for (int timestep = 1; timestep < lipm.timesteps + 1; timestep++)
  {
    // Ensuring ZMP remains in the support polygon
    auto& zmp_constraint = problem.add_constraint(PolygonConstraint::in_polygon_xy(
        lipm.zmp(timestep, omega_2), support.support_polygon(), parameters.zmp_margin));
    if (soft)
    {
      zmp_constraint.configure(ProblemConstraint::Soft, zmp_in_support_weight);
    }

    // Optional offset for single supports
    double x_offset = 0., y_offset = 0.;
    if (!support.is_both())
    {
      x_offset = parameters.foot_zmp_target_x;
      y_offset = (support.side() == HumanoidRobot::Left) ? parameters.foot_zmp_target_y : -parameters.foot_zmp_target_y;
    }

    // ZMP reference trajectory : target is the center of the support polygon
    Eigen::Vector2d zmp_target = (support.frame() * Eigen::Vector3d(x_offset, y_offset, 0)).head(2);
    problem.add_constraint(lipm.zmp(timestep, omega_2) == zmp_target)
        .configure(ProblemConstraint::Soft, parameters.zmp_reference_weight);

    // At the end of an end support, we reach the target with a null speed and a null acceleration
    if (support.end && timestep == lipm.timesteps)
    {
      auto& pos_constraint = problem.add_constraint(
          lipm.pos(timestep) == Eigen::Vector2d(support.frame().translation().x(), support.frame().translation().y()));
      auto& vel_constraint = problem.add_constraint(lipm.vel(timestep) == Eigen::Vector2d(0., 0.));
      auto& acc_constraint = problem.add_constraint(lipm.acc(timestep) == Eigen::Vector2d(0., 0.));

      if (soft)
      {
        pos_constraint.configure(ProblemConstraint::Soft, stop_end_support_weight);
        vel_constraint.configure(ProblemConstraint::Soft, stop_end_support_weight);
        acc_constraint.configure(ProblemConstraint::Soft, stop_end_support_weight);
      }
    }
  }
}

bool WalkPatternGenerator::plan_com(Trajectory& trajectory, std::vector<FootstepsPlanner::Support>& supports,
                                    Eigen::Vector2d initial_pos, Eigen::Vector2d initial_vel,
                                    Eigen::Vector2d initial_acc)
{
  // Initialization of the LIPM problem
  problem::Problem problem;
  std::vector<LIPM> lipms;

  // Building each TrajectoryPart and LIPM
  double t = trajectory.t_start;
  for (auto& support : supports)
  {
    double support_duration = (1 - support.elapsed_ratio) * support_default_duration(support) * support.time_ratio;
    int lipm_timesteps = std::max(1, int((1 - support.elapsed_ratio) * support_default_timesteps(support)));

    if (support.t_start == -1)
    {
      support.t_start = t;
    }

    TrajectoryPart part(support, t);
    t += support_duration;
    part.t_end = t;

    double lipm_dt = support_duration / lipm_timesteps;
    if (part.t_start == trajectory.t_start)
    {
      lipms.emplace_back(problem, lipm_dt, lipm_timesteps, part.t_start, initial_pos, initial_vel, initial_acc);
    }
    else
    {
      lipms.emplace_back(problem, lipm_dt, lipm_timesteps, part.t_start, lipms.back());
    }
    constrain_lipm(problem, lipms.back(), support, omega_2, parameters);

    trajectory.parts.push_back(part);
  }
  trajectory.t_end = t;

  // Solving the problem
  try
  {
    problem.solve();
  }
  catch (const std::exception& e)
  {
    return false;
  }

  // Attributing the LIPM trajectories to the trajectory parts
  for (int i = 0; i < (int)trajectory.parts.size(); i++)
  {
    auto& part = trajectory.parts[i];
    part.com_trajectory = lipms[i].get_trajectory();
    part.support.target_world_dcm = part.com_trajectory.dcm(part.t_end, omega);
  }

  return true;
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::plan(std::vector<FootstepsPlanner::Support>& supports,
                                                            Eigen::Vector3d initial_com_world, double t_start)
{
  if (supports.size() == 0)
  {
    throw std::runtime_error("Trying to plan() with 0 supports");
  }

  Trajectory trajectory(parameters.walk_com_height, t_start, parameters.walk_trunk_pitch, 0.);

  plan_com(trajectory, supports, initial_com_world.head(2));
  plan_feet_trajectories(trajectory);

  return trajectory;
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::replan(std::vector<FootstepsPlanner::Support>& supports,
                                                              WalkPatternGenerator::Trajectory& old_trajectory,
                                                              double t_replan)
{
  if (supports.size() == 0)
  {
    throw std::runtime_error("Trying to replan() with 0 supports");
  }

  // std::cout << "------------------ Old Next support: -----------------" << std::endl;
  // std::cout << "x: " << old_trajectory.get_next_support(t_replan).frame().translation().x() << std::endl;
  // std::cout << "y: " << old_trajectory.get_next_support(t_replan).frame().translation().y() << std::endl;
  // std::cout << "------------------ New Next support: -----------------" << std::endl;
  // std::cout << "x: " << supports[1].frame().translation().x() << std::endl;
  // std::cout << "y: " << supports[1].frame().translation().y() << std::endl;

  Trajectory trajectory(parameters.walk_com_height, t_replan, parameters.walk_trunk_pitch, 0.);

  Eigen::Vector2d initial_pos = old_trajectory.get_p_world_CoM(t_replan).head(2);
  Eigen::Vector2d initial_vel = old_trajectory.get_v_world_CoM(t_replan).head(2);
  Eigen::Vector2d initial_acc = old_trajectory.get_a_world_CoM(t_replan).head(2);
  bool planned_com = plan_com(trajectory, supports, initial_pos, initial_vel, initial_acc);

  if (!planned_com)
  {
    return old_trajectory;
  }

  plan_feet_trajectories(trajectory, &old_trajectory);

  return trajectory;
}

bool WalkPatternGenerator::can_replan_supports(Trajectory& trajectory, double t_replan)
{
  // We can't replan from an "end", a "start" or if the next support is an "end"
  if (trajectory.get_support(t_replan).end || trajectory.get_support(t_replan).start ||
      trajectory.get_next_support(t_replan).end)
  {
    return false;
  }

  // We can't replan if the current support is a double support
  if (trajectory.get_support(t_replan).is_both())
  {
    return false;
  }

  // XXX: Need to move to a branch
  // We can't replan if more than 1/2 of the support phase has passed
  // if (trajectory.get_support(t_replan).elapsed_ratio > 0.8)
  // {
  //   return false;
  // }
  // if (t_replan - trajectory.get_support(t_replan).t_start < 0.005)
  // {
  //   return false;
  // }

  return true;
}

std::vector<FootstepsPlanner::Support> WalkPatternGenerator::replan_supports(FootstepsPlanner& planner,
                                                                             Trajectory& trajectory, double t_replan,
                                                                             double t_last_replan)
{
  if (!can_replan_supports(trajectory, t_replan))
  {
    throw std::runtime_error("This trajectory can't be replanned for supports (check can_replan_supports() before)");
  }

  FootstepsPlanner::Support current_support = trajectory.get_support(t_replan);
  FootstepsPlanner::Support next_support = trajectory.get_next_support(t_replan);

  HumanoidRobot::Side flying_side;
  if (!current_support.is_both())
  {
    flying_side = current_support.side();
  }
  else
  {
    flying_side = HumanoidRobot::other_side(next_support.side());
  }

  Eigen::Affine3d T_world_left;
  Eigen::Affine3d T_world_right;
  if (flying_side == HumanoidRobot::Left)
  {
    T_world_left = current_support.footstep_frame(HumanoidRobot::Left);
    T_world_right = next_support.footstep_frame(HumanoidRobot::Right);
  }
  if (flying_side == HumanoidRobot::Right)
  {
    T_world_left = next_support.footstep_frame(HumanoidRobot::Left);
    T_world_right = current_support.footstep_frame(HumanoidRobot::Right);
  }
  auto footsteps = planner.plan(flying_side, T_world_left, T_world_right);

  std::vector<FootstepsPlanner::Support> supports;
  if (current_support.is_both())
  {
    supports = FootstepsPlanner::make_supports(footsteps, current_support.t_start, false,
                                               parameters.has_double_support(), true);
    supports.erase(supports.begin());
  }
  else
  {
    supports = FootstepsPlanner::make_supports(footsteps, current_support.t_start, false,
                                               parameters.has_double_support(), true);
  }

  double elapsed_duration = t_replan - std::max(t_last_replan, current_support.t_start);
  supports[0].elapsed_ratio =
      current_support.elapsed_ratio +
      elapsed_duration / (support_default_duration(current_support) * current_support.time_ratio);
  supports[0].time_ratio = current_support.time_ratio;
  return supports;
}

int WalkPatternGenerator::support_default_timesteps(FootstepsPlanner::Support& support)
{
  if (support.is_both())
  {
    if (support.start || support.end)
    {
      return parameters.startend_double_support_timesteps();
    }
    return parameters.double_support_timesteps();
  }
  return parameters.single_support_timesteps;
}

double WalkPatternGenerator::support_default_duration(FootstepsPlanner::Support& support)
{
  if (support.is_both())
  {
    if (support.start || support.end)
    {
      return parameters.startend_double_support_duration();
    }
    return parameters.double_support_duration();
  }
  return parameters.single_support_duration;
}

void WalkPatternGenerator::Trajectory::print_parts_timings()
{
  for (int i = 0; i < (int)parts.size(); i++)
  {
    std::cout << "Part " << i << " : start at " << parts[i].t_start << ", end at " << parts[i].t_end << std::endl;
  }
}

std::vector<FootstepsPlanner::Support> WalkPatternGenerator::update_supports(
    double t, std::vector<FootstepsPlanner::Support> supports, Eigen::Vector2d world_measured_dcm)
{
  FootstepsPlanner::Support current_support = supports[0];
  if (current_support.is_both())
  {
    throw std::runtime_error("Can't modify flying target and step duration if the current support is both");
  }

  placo::problem::Problem problem;
  double w1 = 5;
  double w2 = 100;
  double w3 = 1000;
  double w_viability = 1e6;

  double elapsed_time = t - current_support.t_start;
  Eigen::Matrix2d R_world_support = current_support.frame().rotation().topLeftCorner(2, 2);
  Eigen::Vector2d p_world_support = current_support.frame().translation().head(2);

  // ----------------- Decision variables: ------------------
  // ZMP of the next support expressed in the current support frame
  placo::problem::Variable* support_next_zmp = &problem.add_variable(2);

  // exp(omega * T) where T is the end of the current support
  placo::problem::Variable* tau = &problem.add_variable(1);

  // Offset of the DCM from the ZMP in the current support frame
  placo::problem::Variable* support_dcm_offset = &problem.add_variable(2);

  // ----------------- Objective functions: -----------------
  // Time reference
  double T = support_default_duration(current_support);
  problem.add_constraint(tau->expr() == exp(omega * T)).configure(ProblemConstraint::Soft, w1);

  // ZMP Reference (expressed in the world frame)
  Expression world_next_zmp_expr = p_world_support + R_world_support * support_next_zmp->expr();
  Eigen::Vector2d world_target_zmp = supports[1].frame().translation().head(2);
  problem.add_constraint(world_next_zmp_expr == world_target_zmp).configure(ProblemConstraint::Soft, w2);

  // DCM offset reference (expressed in the world frame)
  Eigen::Vector2d world_target_dcm_offset = current_support.target_world_dcm - world_target_zmp;
  Expression world_dcm_offset_expr = R_world_support * support_dcm_offset->expr();
  problem.add_constraint(world_dcm_offset_expr == world_target_dcm_offset).configure(ProblemConstraint::Soft, w3);

  // --------------------- Constraints: ---------------------
  // Time constraints
  double T_min = std::max(0.15, elapsed_time);  // Should probably add an offset to elapsed_time
  double T_max = std::max(3., elapsed_time);    // Arbitrary value of 3s
  problem.add_constraint(tau->expr() >= exp(omega * (T_min))).configure(ProblemConstraint::Hard);
  problem.add_constraint(tau->expr() <= exp(omega * (T_max))).configure(ProblemConstraint::Hard);

  // ZMP and DCM offset constraints (expressed in the current support frame)
  if (current_support.side() == HumanoidRobot::Side::Right)
  {
    problem.add_constraint(PolygonConstraint::in_polygon_xy(support_next_zmp->expr(), parameters.op_space_polygon))
        .configure(ProblemConstraint::Hard);

    std::vector<Eigen::Vector2d> dcm_offset_polygon = parameters.dcm_offset_polygon;
    for (auto& point : dcm_offset_polygon)
    {
      point(0) = -point(0);
      point(1) = -point(1);
    }
    problem.add_constraint(PolygonConstraint::in_polygon_xy(support_dcm_offset->expr(), parameters.dcm_offset_polygon))
        .configure(ProblemConstraint::Soft, w_viability);
  }
  else
  {
    std::vector<Eigen::Vector2d> op_space_polygon = parameters.op_space_polygon;
    for (auto& point : op_space_polygon)
    {
      point(0) = -point(0);
      point(1) = -point(1);
    }
    problem.add_constraint(PolygonConstraint::in_polygon_xy(support_next_zmp->expr(), op_space_polygon))
        .configure(ProblemConstraint::Hard);

    problem.add_constraint(PolygonConstraint::in_polygon_xy(support_dcm_offset->expr(), parameters.dcm_offset_polygon))
        .configure(ProblemConstraint::Soft, w_viability);
  }

  // LIPM Dynamics (expressed in the world frame)
  double duration = std::max(0., T - elapsed_time);
  Eigen::Vector2d world_virtual_zmp =
      get_optimal_zmp(world_measured_dcm, current_support.target_world_dcm, duration, current_support);
  // Eigen::Vector2d world_virtual_zmp = flatten_on_floor(current_support.frame()).translation().head(2);
  problem
      .add_constraint(world_next_zmp_expr + world_dcm_offset_expr ==
                      (world_measured_dcm - world_virtual_zmp) * exp(-omega * elapsed_time) * tau->expr() +
                          world_virtual_zmp)
      .configure(ProblemConstraint::Hard);

  problem.solve();

  // Updating next support position
  // XXX: Problème de transform T qui s'applique ?
  Eigen::Vector2d world_next_zmp_val = p_world_support + R_world_support * support_next_zmp->value;
  supports[1].footsteps[0].frame.translation().x() = world_next_zmp_val(0);
  supports[1].footsteps[0].frame.translation().y() = world_next_zmp_val(1);

  // Updating current support remaining duration
  double support_remaining_time = log(tau->value(0)) / omega - elapsed_time;
  supports[0].time_ratio =
      support_remaining_time / (support_default_duration(current_support) * (1 - current_support.elapsed_ratio));

  return supports;
}

Eigen::Vector2d WalkPatternGenerator::get_optimal_zmp(Eigen::Vector2d world_dcm_start, Eigen::Vector2d world_dcm_end,
                                                      double duration, FootstepsPlanner::Support& support)
{
  placo::problem::Problem problem;

  // Decision variables
  placo::problem::Variable* world_zmp = &problem.add_variable(2);

  // LIPM Dynamics
  // problem.add_constraint(world_dcm_end == (world_dcm_start - world_zmp->expr()) * exp(omega * duration) +
  // world_zmp->expr()).configure(ProblemConstraint::Soft, 1);
  problem
      .add_constraint(world_zmp->expr() ==
                      (world_dcm_end - world_dcm_start * exp(omega * duration)) / (1 - exp(omega * duration)))
      .configure(ProblemConstraint::Soft, 1);

  // ZMP constrained to stay in the support polygon
  problem
      .add_constraint(
          PolygonConstraint::in_polygon_xy(world_zmp->expr(), support.support_polygon(), parameters.zmp_margin))
      .configure(ProblemConstraint::Hard);

  problem.solve();
  return world_zmp->value;
}
}  // namespace placo::humanoid