#include "placo/humanoid/walk_pattern_generator.h"
#include "placo/humanoid/footsteps_planner.h"
#include "placo/problem/polygon_constraint.h"
#include "placo/tools/utils.h"

namespace placo::humanoid
{
using namespace placo::problem;
using namespace placo::tools;

WalkPatternGenerator::Trajectory::Trajectory() : left_foot_yaw(true), right_foot_yaw(true), trunk_yaw(true)
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
    if (part.kick_part)
    {
      return T * _buildFrame(part.kick_trajectory.pos(t), left_foot_yaw.pos(t));
    }
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
    if (part.kick_part)
    {
      return T * _buildFrame(part.kick_trajectory.pos(t), right_foot_yaw.pos(t));
    }
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
    if (part.kick_part)
    {
      return T.linear() * part.kick_trajectory.vel(t);
    }
    return T.linear() * part.swing_trajectory.vel(t);
  }

  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_v_world_right(double t)
{
  TrajectoryPart& part = _findPart(parts, t);

  if (part.support.side() == HumanoidRobot::Left)
  {
    if (part.kick_part)
    {
      return T.linear() * part.kick_trajectory.vel(t);
    }
    return T.linear() * part.swing_trajectory.vel(t);
  }
  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_p_world_CoM(double t)
{
  Eigen::Vector3d pos = Eigen::Vector3d(com.pos(t).x(), com.pos(t).y(), com_target_z);
  return T * pos;
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_v_world_CoM(double t)
{
  Eigen::Vector3d vel = Eigen::Vector3d(com.vel(t).x(), com.vel(t).y(), 0);
  return T.linear() * vel;
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_a_world_CoM(double t)
{
  Eigen::Vector3d acc = Eigen::Vector3d(com.acc(t).x(), com.acc(t).y(), 0);
  return T.linear() * acc;
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_j_world_CoM(double t)
{
  Eigen::Vector3d jerk = Eigen::Vector3d(com.jerk(t).x(), com.jerk(t).y(), 0);
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
  return T.linear() * Eigen::AngleAxisd(trunk_yaw.pos(t), Eigen::Vector3d::UnitZ()).matrix() *
         Eigen::AngleAxisd(trunk_pitch, Eigen::Vector3d::UnitY()).matrix() *
         Eigen::AngleAxisd(trunk_roll, Eigen::Vector3d::UnitX()).matrix();
}

HumanoidRobot::Side WalkPatternGenerator::Trajectory::support_side(double t)
{
  return _findPart(parts, t).support.side();
}

bool WalkPatternGenerator::Trajectory::support_is_both(double t)
{
  return _findPart(parts, t).support.is_both();
}

placo::tools::CubicSpline& WalkPatternGenerator::Trajectory::yaw(HumanoidRobot::Side side)
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
  TrajectoryPart part = _findPart(parts, t);
  for (int i = 0; i < n; i++)
  {
    part = _findPart(parts, part.t_end + 1e-4);
  }
  return T * part.support;
}

FootstepsPlanner::Support WalkPatternGenerator::Trajectory::get_prev_support(double t, int n)
{
  TrajectoryPart part = _findPart(parts, t);
  for (int i = 0; i < n; i++)
  {
    part = _findPart(parts, part.t_start - 1e-4);
  }
  return T * part.support;
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
  TrajectoryPart part = _findPart(parts, t);
  return part.t_start;
}

double WalkPatternGenerator::Trajectory::get_part_t_end(double t)
{
  TrajectoryPart part = _findPart(parts, t);
  return part.t_end;
}

int WalkPatternGenerator::support_timesteps(FootstepsPlanner::Support& support)
{
  if (support.kick())
  {
    return parameters.kick_support_timesteps();
  }

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
  }
}

// XXX : No more management of the CoM height while kicking
void WalkPatternGenerator::planCoM(Trajectory& trajectory, Eigen::Vector2d initial_pos, Eigen::Vector2d initial_vel,
                                   Eigen::Vector2d initial_acc, Trajectory* old_trajectory, double t_replan)
{
  // Computing how many steps are required
  int timesteps = 0;

  for (size_t i = 0; i < trajectory.supports.size(); i++)
  {
    timesteps += support_timesteps(trajectory.supports[i]);

    // While kicking, we always want to plan the CoM for the next support
    if (trajectory.supports[i].kick())
    {
      i++;
      timesteps += support_timesteps(trajectory.supports[i]);
    }

    if (timesteps >= parameters.planned_timesteps)
    {
      timesteps = parameters.planned_timesteps;
      break;
    }
  }

  trajectory.jerk_planner_timesteps = timesteps;

  // How many timesteps should be kept from the former trajectory
  int kept_timesteps = 0;
  if (trajectory.supports[0].replanned)
  {
    kept_timesteps = int((t_replan - trajectory.t_start) / parameters.dt()) + 1;
  }
  trajectory.kept_ts = kept_timesteps;

  // Creating the planner
  Problem problem = Problem();
  LIPM lipm = LIPM(problem, timesteps, parameters.dt(), initial_pos, initial_vel, initial_acc);
  lipm.t_start = trajectory.t_start;

  // We ensure that the first tile of the old trajectory starts with the same jerks as initially planned
  if (old_trajectory != nullptr)
  {
    for (int timestep = 1; timestep < kept_timesteps; timestep++)
    {
      Eigen::Vector2d jerk = old_trajectory->get_j_world_CoM(trajectory.t_start + timestep * parameters.dt()).head(2);
      problem.add_constraint(lipm.jerk(timestep) == jerk)
          .configure(ProblemConstraint::Soft, 1e-4);
    }
  }

  // Adding ZMP constraint and reference trajectory
  int constrained_timesteps = 0;
  FootstepsPlanner::Support current_support;
  for (size_t i = 0; i < trajectory.supports.size(); i++)
  {
    current_support = trajectory.supports[i];
    int step_timesteps = support_timesteps(current_support);

    for (int timestep = constrained_timesteps; timestep < fmin(timesteps, constrained_timesteps + step_timesteps); timestep++)
    {
      // Ensuring ZMP remains in the support polygon
      if (timestep > kept_timesteps)
      {
        problem.add_constraint(PolygonConstraint::in_polygon_xy(
            lipm.zmp(timestep, omega_2), current_support.support_polygon(), parameters.zmp_margin));
      }
      
      // ZMP reference trajectory : aiming for the center of single supports
      if (!current_support.is_both() || current_support.start || current_support.end)
      {
        double y_offset = 0.;
        if (!current_support.is_both())
        {
          if (current_support.side() == HumanoidRobot::Left)
          {
            if (current_support.kick())
            {
              y_offset = parameters.kick_zmp_target_y;
            }
            else
            {
              y_offset = parameters.foot_zmp_target_y;
            }
          }
          else
          {
            if (current_support.kick())
            {
              y_offset = -parameters.kick_zmp_target_y;
            }
            else
            {
              y_offset = -parameters.foot_zmp_target_y;
            }
          }
        }

        double x_offset = 0.;
        if (current_support.kick())
        {
          x_offset = parameters.kick_zmp_target_x;
        }
        else
        {
          x_offset = parameters.foot_zmp_target_x;
        }

        Eigen::Vector3d zmp_target = current_support.frame() * Eigen::Vector3d(x_offset, y_offset, 0);
        problem.add_constraint(lipm.zmp(timestep, omega_2) == zmp_target.head(2))
            .configure(ProblemConstraint::Soft, parameters.zmp_reference_weight);
      }
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

void WalkPatternGenerator::planKickTrajectory(TrajectoryPart& part, Trajectory& trajectory, int step, double& t)
{
  part.kick_part = true;
  t += parameters.kick_support_duration();

  HumanoidRobot::Side kicking_side = HumanoidRobot::other_side(part.support.side());
  Eigen::Vector3d start = trajectory.supports[step - 1].footstep_frame(kicking_side).translation();
  Eigen::Vector3d target = trajectory.supports[step + 1].footstep_frame(kicking_side).translation();
  Eigen::Affine3d T_world_opposite =
      parameters.opposite_frame(part.support.footsteps[0].side, part.support.footsteps[0].frame);

  part.kick_trajectory = Kick::make_trajectory(kicking_side, t - parameters.kick_support_duration(), t, start, target,
                                               T_world_opposite, parameters);

  // Support foot remaining steady
  trajectory.add_supports(t, part.support);
}

void WalkPatternGenerator::planDoubleSupportTrajectory(TrajectoryPart& part, Trajectory& trajectory, double& t)
{
  if (part.support.start || part.support.end)
  {
    t += parameters.startend_double_support_duration();
  }
  else
  {
    t += parameters.double_support_duration();
  }
  trajectory.add_supports(t, part.support);
  trajectory.trunk_yaw.add_point(t, frame_yaw(part.support.frame().rotation()), 0);
}

void WalkPatternGenerator::planSingleSupportTrajectory(TrajectoryPart& part, Trajectory& trajectory, int step,
                                                       double& t, Trajectory* old_trajectory, double t_replan)
{
  // Single support, add the flying trajectory
  HumanoidRobot::Side flying_side = HumanoidRobot::other_side(part.support.footsteps[0].side);

  // Computing the intermediary flying foot inflection target
  Eigen::Affine3d T_world_flyingTarget = trajectory.supports[step + 1].footstep_frame(flying_side);

  t += parameters.single_support_duration;

  // Current step case
  if (part.support.replanned)
  {
    auto& old_part = _findPart(old_trajectory->parts, t_replan);

    part.swing_trajectory = SwingFootCubic::make_trajectory(
        old_part.t_start, old_part.t_end, parameters.walk_foot_height, parameters.walk_foot_rise_ratio,
        old_trajectory->T * old_part.swing_trajectory.pos(old_part.t_start),
        old_trajectory->T * old_part.swing_trajectory.pos(old_part.t_end));
  }

  // Complete steps case
  else
  {
    Eigen::Affine3d T_world_startTarget = trajectory.supports[step - 1].footstep_frame(flying_side);

    // Flying foot reaching its position
    part.swing_trajectory = SwingFootCubic::make_trajectory(
        t - parameters.single_support_duration, t, parameters.walk_foot_height, parameters.walk_foot_rise_ratio,
        T_world_startTarget.translation(), T_world_flyingTarget.translation());
  }

  trajectory.yaw(flying_side).add_point(t, frame_yaw(T_world_flyingTarget.rotation()), 0);

  // The trunk orientation follow the steps orientation
  trajectory.trunk_yaw.add_point(t, frame_yaw(T_world_flyingTarget.rotation()), 0);

  // Support foot remaining steady
  trajectory.add_supports(t, part.support);
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
    trajectory.yaw(side).add_point(t, placo::tools::frame_yaw(old_trajectory->get_T_world_foot(side, t).linear()), 0);
  }

  for (size_t step = 0; step < trajectory.supports.size(); step++)
  {
    auto& support = trajectory.supports[step];

    TrajectoryPart part;
    part.support = support;
    part.t_start = t;

    // Single support
    if (support.footsteps.size() == 1)
    {
      if (support.kick())
      {
        planKickTrajectory(part, trajectory, step, t);
      }
      else
      {
        planSingleSupportTrajectory(part, trajectory, step, t, old_trajectory, t_replan);
      }
    }

    // Double support
    else
    {
      planDoubleSupportTrajectory(part, trajectory, t);
    }

    part.t_end = t;
    trajectory.parts.push_back(part);
  }

  trajectory.t_end = t;
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::plan(std::vector<FootstepsPlanner::Support>& supports,
                                                            Eigen::Vector3d initial_com_world, double t_start)
{
  if (supports.size() == 0)
  {
    throw std::runtime_error("Trying to plan() with 0 supports");
  }

  // Initialization of the trajectory using the minimum walking height
  Trajectory trajectory;
  trajectory.t_start = t_start;
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;
  trajectory.supports = supports;
  trajectory.com_target_z = parameters.walk_com_height;

  // Planning the center of mass trajectory
  planCoM(trajectory, initial_com_world.head(2));

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
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;
  trajectory.com_target_z = parameters.walk_com_height;
  trajectory.supports = supports;
  trajectory.t_start = old_trajectory.get_part_t_start(t_replan);

  // Planning the center of mass trajectory
  Eigen::Vector2d com_pos = old_trajectory.get_p_world_CoM(trajectory.t_start).head(2);
  Eigen::Vector2d com_vel = old_trajectory.get_v_world_CoM(trajectory.t_start).head(2);
  Eigen::Vector2d com_acc = old_trajectory.get_a_world_CoM(trajectory.t_start).head(2);
  planCoM(trajectory, com_pos, com_vel, com_acc, &old_trajectory, t_replan);

  // Planning the footsteps trajectories
  planFeetTrajectories(trajectory, &old_trajectory, t_replan);

  return trajectory;
}

bool WalkPatternGenerator::can_replan_supports(Trajectory& trajectory, double t_replan)
{
  // We can't replan from an "end", a "start" or a "kick"
  if (trajectory.get_support(t_replan).end || trajectory.get_support(t_replan).start || 
      trajectory.get_next_support(t_replan).end || trajectory.get_support(t_replan).kick())
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

  FootstepsPlanner::Support current_support = trajectory.get_support(t_replan);
  FootstepsPlanner::Support next_support = trajectory.get_next_support(t_replan);

  Eigen::Affine3d T_world_left;
  Eigen::Affine3d T_world_right;
  HumanoidRobot::Side flying_side;

  if (!current_support.is_both())
  {
    flying_side = current_support.side();
  }
  else
  {
    flying_side = HumanoidRobot::other_side(next_support.side());
  }

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
  supports = FootstepsPlanner::make_supports(footsteps, false, parameters.has_double_support(), true);
  
  if (current_support.is_both())
  {
    supports.erase(supports.begin());
  }

  supports[0].replanned = true;
  return supports;
}
}  // namespace placo::humanoid