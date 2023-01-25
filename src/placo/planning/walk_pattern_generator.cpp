#include "placo/planning/walk_pattern_generator.h"
#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/utils.h"

namespace placo
{
WalkPatternGenerator::WalkPatternGenerator(HumanoidRobot& robot) : robot(robot)
{
}

void WalkPatternGenerator::planFootsteps(Trajectory& trajectory, Eigen::Affine3d T_world_left,
                                         Eigen::Affine3d T_world_right, Eigen::Affine3d T_world_targetLeft,
                                         Eigen::Affine3d T_world_targetRight)
{
  // Creates the planner and run the planning
  FootstepsPlannerNaive planner(robot.support_side, T_world_left, T_world_right);
  planner.parameters = parameters;

  auto footsteps = planner.plan(T_world_targetLeft, T_world_targetRight);

  int dsp_steps = std::round(parameters.double_support_duration / parameters.dt);

  trajectory.footsteps = planner.make_double_supports(footsteps, true, dsp_steps > 0, true);
}

void WalkPatternGenerator::planCoM(Trajectory& trajectory)
{
  // Test if the first support is a single support
  int first_is_single = (trajectory.footsteps[0].footsteps.size() == 1) ? 1 : 0;

  // Computing how many steps are required
  int ssp_steps = std::round(parameters.single_support_duration / parameters.dt);
  int dsp_steps = std::round(parameters.double_support_duration / parameters.dt);
  int se_dsp_steps = std::round(parameters.startend_double_support_duration / parameters.dt);
  int total_steps = 0;

  for (auto& support : trajectory.footsteps)
  {
    if (support.footsteps.size() == 1 && support.start_end == false)
    {
      total_steps += ssp_steps;
    }
    else if (support.footsteps.size() == 2)
    {
      if (support.start_end)
      {
        total_steps += se_dsp_steps;
      }
      else
      {
        total_steps += dsp_steps;
      }
    }

    if (total_steps >= parameters.maximum_steps)
    {
      break;
    }
  }

  auto com_world = robot.com_world();
  trajectory.jerk_planner_steps = total_steps;

  // Creating the planner
  JerkPlanner planner(total_steps, Eigen::Vector2d(com_world.x(), com_world.y()), Eigen::Vector2d(0., 0.),
                      Eigen::Vector2d(0., 0.), parameters.dt, parameters.omega());

  // Adding constraints
  bool double_support = (parameters.double_support_duration / parameters.dt) > 1;
  int steps = 0;
  for (int i = 0; i < trajectory.footsteps.size(); i++)
  {
    auto support = trajectory.footsteps[i];
    if (support.footsteps.size() == 1 && support.start_end == false)
    {
      // XXX: To investigate
      for (int k = 0; k < ssp_steps; k++)
      {
        if (k == (ssp_steps / 2))
        {
          // Adding a constraint at the begining of the stem
          // planner.add_polygon_constraint(steps + k, support.support_polygon(), JerkPlanner::ZMP,
          // parameters.zmp_margin);
          auto target = support.frame().translation();
          planner.add_equality_constraint(steps + k + first_is_single, Eigen::Vector2d(target.x(), target.y()),
                                          JerkPlanner::ZMP);
        }
      }

      steps += ssp_steps;

      // Adding a constraint to have the ZMP between the 2 feet at the end of
      // the step if there is no double support phase
      if (!double_support)
      {
        auto target = (support.frame().translation() + trajectory.footsteps[i + 1].frame().translation()) / 2;
        planner.add_equality_constraint(steps, Eigen::Vector2d(target.x(), target.y()), JerkPlanner::ZMP);
      }
    }

    else if (support.footsteps.size() == 2)
    {
      if (support.start_end)
      {
        steps += se_dsp_steps;
      }
      else
      {
        steps += dsp_steps;
      }
    }

    // We reach the target with the given position, a null speed and a null acceleration
    if (steps >= total_steps)
    {
      auto frame = support.frame();
      planner.add_equality_constraint(
          total_steps - 1, Eigen::Vector2d(frame.translation().x(), frame.translation().y()), JerkPlanner::Position);
      planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(0., 0.), JerkPlanner::Velocity);
      planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(0., 0.), JerkPlanner::Acceleration);

      break;
    }
  }

  trajectory.com = planner.plan();
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

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_left(double t)
{
  TrajectoryPart& part = _findPart(parts, t);

  if (part.support.side() == HumanoidRobot::Right)
  {
    return _buildFrame(part.swing_trajectory.pos(t), left_foot_yaw.get(t));
  }
  else
  {
    return _buildFrame(part.support.frame(HumanoidRobot::Left).translation(), left_foot_yaw.get(t));
  }
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_right(double t)
{
  TrajectoryPart& part = _findPart(parts, t);

  if (part.support.side() == HumanoidRobot::Left)
  {
    return _buildFrame(part.swing_trajectory.pos(t), right_foot_yaw.get(t));
  }
  else
  {
    return _buildFrame(part.support.frame(HumanoidRobot::Right).translation(), right_foot_yaw.get(t));
  }
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_CoM_world(double t)
{
  auto pos = com.pos(t);

  return Eigen::Vector3d(pos.x(), pos.y(), com_height);
}

Eigen::Matrix3d WalkPatternGenerator::Trajectory::get_R_world_trunk(double t)
{
  return Eigen::AngleAxisd(trunk_yaw.get(t), Eigen::Vector3d::UnitZ()).matrix() *
         Eigen::AngleAxisd(trunk_pitch, Eigen::Vector3d::UnitY()).matrix();
}

HumanoidRobot::Side WalkPatternGenerator::Trajectory::support_side(double t)
{
  return _findPart(parts, t).support.side();
}

rhoban_utils::PolySpline& WalkPatternGenerator::Trajectory::yaw(HumanoidRobot::Side side)
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

static void _addSupports(WalkPatternGenerator::Trajectory& trajectory, double t, FootstepsPlanner::Support& support)
{
  for (auto footstep : support.footsteps)
  {
    auto T_world_foot = footstep.frame;
    trajectory.yaw(footstep.side).addPoint(t, frame_yaw(T_world_foot.rotation()), 0);
  }
}

void WalkPatternGenerator::planFeetTrajectories(Trajectory& trajectory)
{
  double t = 0.0;

  // First, adds the initial position to the trajectory
  _addSupports(trajectory, 0., trajectory.footsteps[0]);
  trajectory.trunk_yaw.addPoint(0, frame_yaw(trajectory.footsteps[0].frame().rotation()), 0);

  for (int step = 0; step < trajectory.footsteps.size(); step++)
  {
    auto& support = trajectory.footsteps[step];

    TrajectoryPart part;
    part.support = support;
    part.t_start = t;

    if (support.footsteps.size() == 1 && support.start_end == false)
    {
      // Single support, add the flying trajectory
      auto flying_side = HumanoidRobot::other_side(support.footsteps[0].side);

      // Computing the intermediary flying foot inflection target
      auto T_world_startTarget = trajectory.footsteps[step - 1].frame(flying_side);
      auto T_world_flyingTarget = trajectory.footsteps[step + 1].frame(flying_side);

      t += parameters.single_support_duration;

      // Flying foot reaching its position
      part.swing_trajectory =
          SwingFoot::make_trajectory(t - parameters.single_support_duration, t, parameters.walk_foot_height,
                                     T_world_startTarget.translation(), T_world_flyingTarget.translation());

      trajectory.yaw(flying_side).addPoint(t, frame_yaw(T_world_flyingTarget.rotation()), 0);
      trajectory.trunk_yaw.addPoint(t, frame_yaw(T_world_flyingTarget.rotation()), 0);

      // Support foot remaining steady
      _addSupports(trajectory, t, support);
    }
    else if (support.footsteps.size() == 2)
    {
      // Double support, adding the support foot at the begining and at the end of the trajectory
      if (support.start_end)
      {
        t += parameters.startend_double_support_duration;
      }
      else
      {
        t += parameters.double_support_duration;
      }
      _addSupports(trajectory, t, support);
      trajectory.trunk_yaw.addPoint(t, frame_yaw(support.frame().rotation()), 0);
    }

    part.t_end = t;
    trajectory.parts.push_back(part);
  }

  trajectory.duration = t;
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right,
                                                            Eigen::Affine3d T_world_targetLeft,
                                                            Eigen::Affine3d T_world_targetRight)
{
  Trajectory trajectory;
  trajectory.com_height = parameters.walk_com_height;
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;

  // Planning the footsteps to take
  planFootsteps(trajectory, T_world_left, T_world_right, T_world_targetLeft, T_world_targetRight);

  // Planning the center of mass trajectory
  planCoM(trajectory);

  // Planning the footsteps trajectories
  planFeetTrajectories(trajectory);

  return trajectory;
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::plan(std::vector<FootstepsPlanner::Support> footsteps)
{
  Trajectory trajectory;
  trajectory.com_height = parameters.walk_com_height;
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;
  trajectory.footsteps = footsteps;

  // Planning the center of mass trajectory
  planCoM(trajectory);

  // Planning the footsteps trajectories
  planFeetTrajectories(trajectory);

  return trajectory;
}

// For python binding
WalkPatternGenerator::Trajectory WalkPatternGenerator::plan_by_frames(Eigen::Affine3d T_world_left,
                                                                      Eigen::Affine3d T_world_right,
                                                                      Eigen::Affine3d T_world_targetLeft,
                                                                      Eigen::Affine3d T_world_targetRight)
{
  return plan(T_world_left, T_world_right, T_world_targetLeft, T_world_targetRight);
}

WalkPatternGenerator::Trajectory
WalkPatternGenerator::plan_by_supports(std::vector<FootstepsPlanner::Support> footsteps)
{
  return plan(footsteps);
}

std::vector<Eigen::Affine3d> WalkPatternGenerator::Trajectory::get_last_footsteps(double t, bool double_support)
{
  TrajectoryPart& part_current = _findPart(parts, t);

  if (double_support)
  {
    return { part_current.support.footsteps[0].frame, part_current.support.footsteps[1].frame };
  }

  TrajectoryPart& part_prev = _findPart(parts, t - 0.025);  // TO CHANGE

  return { part_prev.support.footsteps[0].frame, part_current.support.footsteps[0].frame };
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_last_footstep(double t, bool double_support)
{
  auto footsteps = get_last_footsteps(t, double_support);
  return footsteps[1];
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_last_last_footstep(double t, bool double_support)
{
  auto footsteps = get_last_footsteps(t, double_support);
  return footsteps[0];
}
}  // namespace placo