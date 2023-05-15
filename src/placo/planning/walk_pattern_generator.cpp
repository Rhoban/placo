#include "placo/planning/walk_pattern_generator.h"
#include "placo/footsteps/footsteps_planner.h"
#include "placo/problem/polygon_constraint.h"
#include "placo/utils.h"

namespace placo
{
WalkPatternGenerator::Trajectory::Trajectory() : left_foot_yaw(true), right_foot_yaw(true), trunk_yaw(true)
{
  T.setIdentity();
}

WalkPatternGenerator::WalkPatternGenerator(HumanoidRobot& robot, HumanoidParameters& parameters)
  : robot(robot), parameters(parameters)
{
}

static Eigen::Affine3d _buildFrame(Eigen::Vector3d position, double orientation)
{
  Eigen::Affine3d frame = Eigen::Affine3d::Identity();

  frame.translation() = position;
  frame.linear() = Eigen::AngleAxisd(orientation, Eigen::Vector3d::UnitZ()).matrix();

  return frame;
}

static WalkPatternGenerator::TrajectoryPart& _findPart(std::vector<WalkPatternGenerator::TrajectoryPart>& parts,
                                                       double t)
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
      return part;
    }
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
  else
  {
    return T * _buildFrame(part.support.footstep_frame(HumanoidRobot::Left).translation(), left_foot_yaw.pos(t));
  }
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_right(double t)
{
  TrajectoryPart& part = _findPart(parts, t);

  if (is_flying(HumanoidRobot::Right, t))
  {
    return T * _buildFrame(part.swing_trajectory.pos(t), right_foot_yaw.pos(t));
  }
  else
  {
    return T * _buildFrame(part.support.footstep_frame(HumanoidRobot::Right).translation(), right_foot_yaw.pos(t));
  }
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

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_p_world_CoM(double t)
{
  auto pos = com.pos(t);

  return T * Eigen::Vector3d(pos.x(), pos.y(), com_height);
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_v_world_CoM(double t)
{
  auto pos = com.vel(t);

  return T.linear() * Eigen::Vector3d(pos.x(), pos.y(), 0);
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_a_world_CoM(double t)
{
  auto pos = com.acc(t);

  return T.linear() * Eigen::Vector3d(pos.x(), pos.y(), 0);
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_j_world_CoM(double t)
{
  auto pos = com.jerk(t);

  return T.linear() * Eigen::Vector3d(pos.x(), pos.y(), 0);
}

Eigen::Matrix3d WalkPatternGenerator::Trajectory::get_R_world_trunk(double t)
{
  return T.linear() * Eigen::AngleAxisd(trunk_yaw.pos(t), Eigen::Vector3d::UnitZ()).matrix() *
         Eigen::AngleAxisd(trunk_pitch, Eigen::Vector3d::UnitY()).matrix();
}

HumanoidRobot::Side WalkPatternGenerator::Trajectory::support_side(double t)
{
  return _findPart(parts, t).support.side();
}

bool WalkPatternGenerator::Trajectory::support_is_both(double t)
{
  return _findPart(parts, t).support.is_both();
}

placo::CubicSpline& WalkPatternGenerator::Trajectory::yaw(HumanoidRobot::Side side)
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

FootstepsPlanner::Support WalkPatternGenerator::Trajectory::get_next_support(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  TrajectoryPart& next_part = _findPart(parts, part.t_end + 1e-4);
  return T * next_part.support;
}

FootstepsPlanner::Support WalkPatternGenerator::Trajectory::get_prev_support(double t)
{
  TrajectoryPart& part = _findPart(parts, t);
  TrajectoryPart& prev_part = _findPart(parts, part.t_start - 1e-4);
  return T * prev_part.support;
}

std::vector<FootstepsPlanner::Support> WalkPatternGenerator::Trajectory::get_supports()
{
  std::vector<FootstepsPlanner::Support> new_supports = supports;

  for (int k = 0; k < new_supports.size(); k++)
  {
    new_supports[k] = T * new_supports[k];
  }

  return new_supports;
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

int WalkPatternGenerator::support_timesteps(FootstepsPlanner::Support& support)
{
  if (support.footsteps.size() == 1)
  {
    return parameters.single_support_timesteps;
  }

  if (support.start || support.end)
  {
    return parameters.startend_double_support_timesteps();
  }
  else
  {
    return parameters.double_support_timesteps();
    ;
  }
}

void WalkPatternGenerator::planCoM(Trajectory& trajectory, Eigen::Vector2d initial_pos, Eigen::Vector2d initial_vel,
                                   Eigen::Vector2d initial_acc, Trajectory* old_trajectory, double t_replan)
{
  // Computing how many steps are required
  int timesteps = 0;

  for (size_t i = 0; i < trajectory.supports.size(); i++)
  {
    timesteps += support_timesteps(trajectory.supports[i]);

    if (timesteps >= parameters.planned_timesteps)
    {
      break;
    }
  }

  trajectory.jerk_planner_timesteps = timesteps;

  // How many timesteps should be kept from the former trajectory
  int kept_timesteps = std::round((t_replan - trajectory.t_start) / parameters.dt());

  // Creating the planner
  Problem problem = Problem();
  LIPM lipm = LIPM(problem, timesteps, parameters.omega(), parameters.dt(), initial_pos, initial_vel, initial_acc);
  lipm.t_start = trajectory.t_start;

  // We ensure that the first tile of the old trajectory starts with the same jerks as initially planned
  if (old_trajectory != nullptr)
  {
    for (int timestep = 0; timestep < kept_timesteps; timestep++)
    {
      Eigen::Vector3d jerk = old_trajectory->get_j_world_CoM(trajectory.t_start + timestep * parameters.dt() + 1e-6);
      problem.add_constraint(lipm.jerk(timestep) == Eigen::Vector2d(jerk.x(), jerk.y()));
    }
  }

  // Adding ZMP constraint and reference trajectory
  int constrained_timesteps = 0;
  FootstepsPlanner::Support current_support;
  for (size_t i = 0; i < trajectory.supports.size(); i++)
  {
    current_support = trajectory.supports[i];
    int step_timesteps = support_timesteps(current_support);

    for (int timestep = constrained_timesteps; timestep < constrained_timesteps + step_timesteps; timestep++)
    {
      // Ensuring ZMP remains in the support polygon
      if (timestep > kept_timesteps)
      {
        problem.add_constraints(PolygonConstraint::in_polygon_xy(lipm.zmp(timestep), current_support.support_polygon(),
                                                                 parameters.zmp_margin));
      }

      // ZMP reference trajectory
      double y_offset = 0.;

      if (!current_support.is_both())
      {
        if (current_support.side() == HumanoidRobot::Left)
        {
          y_offset = parameters.foot_zmp_target_y;
        }
        else
        {
          y_offset = -parameters.foot_zmp_target_y;
        }
      }

      Eigen::Vector3d zmp_target = current_support.frame() * Eigen::Vector3d(parameters.foot_zmp_target_x, y_offset, 0);

      problem.add_constraint(lipm.zmp(timestep) == Eigen::Vector2d(zmp_target.x(), zmp_target.y()))
          .configure(ProblemConstraint::Soft, 1e-1);
    }

    constrained_timesteps += step_timesteps;

    if (constrained_timesteps >= timesteps)
    {
      break;
    }
  }

  // We reach the target with the given position, a null speed and a null acceleration
  if (current_support.end)
  {
    // XXX: In the case we are not on an "end", maybe we want to target another final condition
    problem.add_constraint(lipm.pos(timesteps) == Eigen::Vector2d(current_support.frame().translation().x(),
                                                                  current_support.frame().translation().y()));
    problem.add_constraint(lipm.vel(timesteps) == Eigen::Vector2d(0., 0.));
    problem.add_constraint(lipm.acc(timesteps) == Eigen::Vector2d(0., 0.));
  }

  problem.solve();
  trajectory.com = lipm.get_trajectory();
}

void WalkPatternGenerator::Trajectory::add_supports(double t, FootstepsPlanner::Support& support)
{
  for (auto footstep : support.footsteps)
  {
    auto T_world_foot = footstep.frame;
    yaw(footstep.side).add_point(t, frame_yaw(T_world_foot.rotation()), 0);
  }
}

void WalkPatternGenerator::planFeetTrajectories(Trajectory& trajectory, Trajectory* old_trajectory, double t_replan)
{
  double t = trajectory.t_start;

  // Add the initial position to the trajectory
  trajectory.add_supports(t, trajectory.supports[0]);

  trajectory.trunk_yaw.add_point(t, frame_yaw(trajectory.supports[0].frame().rotation()), 0);

  if (!trajectory.supports[0].is_both())
  {
    if (old_trajectory == nullptr)
    {
      throw std::runtime_error("Can't replan a swing foot starting with a single support");
    }

    // Retrieving initial flying foot yaw from old trajectory
    HumanoidRobot::Side side = HumanoidRobot::other_side(trajectory.supports[0].side());
    trajectory.yaw(side).add_point(t, placo::frame_yaw(old_trajectory->get_T_world_foot(side, t).linear()), 0);
  }

  for (size_t step = 0; step < trajectory.supports.size(); step++)
  {
    auto& support = trajectory.supports[step];

    TrajectoryPart part;
    part.support = support;
    part.t_start = t;

    if (support.footsteps.size() == 1)
    {
      // Single support, add the flying trajectory
      HumanoidRobot::Side flying_side = HumanoidRobot::other_side(support.footsteps[0].side);

      // Computing the intermediary flying foot inflection target
      Eigen::Affine3d T_world_flyingTarget = trajectory.supports[step + 1].footstep_frame(flying_side);

      t += parameters.single_support_duration;

      if (support.start)
      {
        auto& old_part = _findPart(old_trajectory->parts, t_replan);

        part.swing_trajectory = WalkSwingFoot::make_trajectory(
            old_part.t_start, old_part.t_end, parameters.walk_foot_height, parameters.walk_foot_rise_ratio,
            old_trajectory->T * old_part.swing_trajectory.pos(old_part.t_start),
            old_trajectory->T * old_part.swing_trajectory.pos(old_part.t_end));
      }
      else
      {
        Eigen::Affine3d T_world_startTarget = trajectory.supports[step - 1].footstep_frame(flying_side);

        // Flying foot reaching its position
        part.swing_trajectory = WalkSwingFoot::make_trajectory(
            t - parameters.single_support_duration, t, parameters.walk_foot_height, parameters.walk_foot_rise_ratio,
            T_world_startTarget.translation(), T_world_flyingTarget.translation());
      }

      trajectory.yaw(flying_side).add_point(t, frame_yaw(T_world_flyingTarget.rotation()), 0);

      // The trunk orientation follow the steps orientation if there isn't double support phases
      // If there is double support phases, it follow the double supports orientation
      if (parameters.double_support_duration() < parameters.dt())
      {
        trajectory.trunk_yaw.add_point(t, frame_yaw(T_world_flyingTarget.rotation()), 0);
      }

      // Support foot remaining steady
      trajectory.add_supports(t, support);
    }

    else
    {
      // Double support, adding the support foot at the begining and at the end of the trajectory
      if (support.start || support.end)
      {
        t += parameters.startend_double_support_duration();
      }
      else
      {
        t += parameters.double_support_duration();
      }
      trajectory.add_supports(t, support);
      trajectory.trunk_yaw.add_point(t, frame_yaw(support.frame().rotation()), 0);
    }

    part.t_end = t;
    trajectory.parts.push_back(part);
  }

  trajectory.t_end = t;
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::plan(std::vector<FootstepsPlanner::Support>& supports,
                                                            double t_start)
{
  if (supports.size() == 0)
  {
    throw std::runtime_error("Trying to plan() with 0 supports");
  }

  // Initialization of the trajectory
  Trajectory trajectory;
  trajectory.t_start = t_start;
  trajectory.com_height = parameters.walk_com_height;
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;
  trajectory.supports = supports;

  // Planning the center of mass trajectory
  auto com_world = robot.com_world();
  planCoM(trajectory, Eigen::Vector2d(com_world.x(), com_world.y()));

  // Planning the footsteps trajectories
  planFeetTrajectories(trajectory);

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

  // Initialization of the new trajectory
  Trajectory trajectory;
  trajectory.com_height = parameters.walk_com_height;
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;
  trajectory.supports = supports;
  trajectory.t_start = old_trajectory.get_part_t_start(t_replan);

  // Planning the center of mass trajectory
  Eigen::Vector3d com_pos = old_trajectory.get_p_world_CoM(trajectory.t_start);
  Eigen::Vector3d com_vel = old_trajectory.get_v_world_CoM(trajectory.t_start);
  Eigen::Vector3d com_acc = old_trajectory.get_a_world_CoM(trajectory.t_start);

  planCoM(trajectory, Eigen::Vector2d(com_pos.x(), com_pos.y()), Eigen::Vector2d(com_vel.x(), com_vel.y()),
          Eigen::Vector2d(com_acc.x(), com_acc.y()), &old_trajectory, t_replan);

  // Planning the footsteps trajectories
  planFeetTrajectories(trajectory, &old_trajectory, t_replan);

  return trajectory;
}

bool WalkPatternGenerator::can_replan_supports(Trajectory& trajectory, double t_replan)
{
  // We can't replan from an "end"
  if (trajectory.get_support(t_replan).end)
  {
    return false;
  }

  // We can't replan if the support and the next support are not both single supports
  placo::FootstepsPlanner::Support current_support = trajectory.get_support(t_replan);
  placo::FootstepsPlanner::Support next_support = trajectory.get_next_support(t_replan);

  if (current_support.is_both() || next_support.is_both())
  {
    return false;
  }

  return true;
}

std::vector<FootstepsPlanner::Support> WalkPatternGenerator::replan_supports(FootstepsPlanner& planner,
                                                                             Trajectory& trajectory, double t_replan)
{
  if (!can_replan_supports(trajectory, t_replan))
  {
    throw std::runtime_error("This trajectory can't be replanned for supports (check can_replan_supports() before)");
  }

  placo::FootstepsPlanner::Support current_support = trajectory.get_support(t_replan);
  placo::FootstepsPlanner::Support next_support = trajectory.get_next_support(t_replan);

  placo::HumanoidRobot::Side flying_side = current_support.side();

  Eigen::Affine3d T_world_left = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_world_right = Eigen::Affine3d::Identity();

  if (flying_side == placo::HumanoidRobot::Left)
  {
    T_world_left = current_support.footstep_frame(placo::HumanoidRobot::Left);
    T_world_right = next_support.footstep_frame(placo::HumanoidRobot::Right);
  }
  if (flying_side == placo::HumanoidRobot::Right)
  {
    T_world_left = next_support.footstep_frame(placo::HumanoidRobot::Left);
    T_world_right = current_support.footstep_frame(placo::HumanoidRobot::Right);
  }
  auto footsteps = planner.plan(flying_side, T_world_left, T_world_right);

  std::vector<FootstepsPlanner::Support> supports;
  supports = placo::FootstepsPlanner::make_supports(footsteps, false, false, true);

  return supports;
}
}  // namespace placo