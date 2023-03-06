#include "placo/planning/walk_pattern_generator.h"
#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/footsteps/footsteps_planner_repetitive.h"
#include "placo/utils.h"

namespace placo
{
WalkPatternGenerator::WalkPatternGenerator(HumanoidRobot& robot, FootstepsPlanner& footsteps_planner,
                                           HumanoidParameters& parameters)
  : robot(robot), footsteps_planner(footsteps_planner), parameters(parameters)
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

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_last_footstep_frame(HumanoidRobot::Side side, double t)
{
  // Search in the current segment of the trajectory
  TrajectoryPart& part = _findPart(parts, t);
  for (auto footstep : part.support.footsteps)
  {
    if (footstep.side == side)
    {
      return footstep.frame;
    }
  }

  // Search in the previous segments of trajectory
  double previous_time = part.t_start - 1e-4;
  while (previous_time > 0)
  {
    auto previous_part = _findPart(parts, previous_time);
    for (auto footstep : previous_part.support.footsteps)
    {
      if (footstep.side == side)
      {
        return footstep.frame;
      }
    }
    previous_time = previous_part.t_start - 1e-4;
  }

  throw std::logic_error("Didn't find a previous footstep having this side");
}

static void _addSupports(WalkPatternGenerator::Trajectory& trajectory, double t, FootstepsPlanner::Support& support)
{
  for (auto footstep : support.footsteps)
  {
    auto T_world_foot = footstep.frame;
    trajectory.yaw(footstep.side).addPoint(t, frame_yaw(T_world_foot.rotation()), 0);
  }
}

void WalkPatternGenerator::planCoM(Trajectory& trajectory, Eigen::Vector2d initial_vel, Eigen::Vector2d initial_acc)
{
  // Computing how many steps are required
  int ssp_steps = std::round(parameters.single_support_duration / parameters.dt);
  int dsp_steps = std::round(parameters.double_support_duration / parameters.dt);
  int se_dsp_steps = std::round(parameters.startend_double_support_duration / parameters.dt);
  int total_steps = 0;

  for (auto& support : trajectory.supports)
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

  trajectory.jerk_planner_steps = total_steps;

  // Creating the planner
  auto com_world = robot.com_world();
  JerkPlanner planner(total_steps, Eigen::Vector2d(com_world.x(), com_world.y()), initial_vel, initial_acc,
                      parameters.dt, parameters.omega());

  // Adding constraints
  int steps = 0;
  for (int i = 0; i < trajectory.supports.size(); i++)
  {
    auto support = trajectory.supports[i];
    if (support.footsteps.size() == 1 && support.start_end == false)
    {
      // XXX: To investigate
      for (int k = 0; k < ssp_steps; k++)
      {
        planner.add_polygon_constraint(steps + k, support.support_polygon(), JerkPlanner::ZMP, 0.03);
        // if (k == (ssp_steps / 2))
        // {
        //   auto target = support.frame().translation();
        //   planner.add_equality_constraint(steps + k, Eigen::Vector2d(target.x(), target.y()), JerkPlanner::ZMP);
        // }
      }

      steps += ssp_steps;
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

void WalkPatternGenerator::planFeetTrajectories(Trajectory& trajectory)
{
  double t = 0.0;

  // First, adds the initial position to the trajectory
  _addSupports(trajectory, 0., trajectory.supports[0]);
  trajectory.trunk_yaw.addPoint(0, frame_yaw(trajectory.supports[0].frame().rotation()), 0);

  for (int step = 0; step < trajectory.supports.size(); step++)
  {
    auto& support = trajectory.supports[step];

    TrajectoryPart part;
    part.support = support;
    part.t_start = t;

    if (support.footsteps.size() == 1 && support.start_end == false)
    {
      // Single support, add the flying trajectory
      auto flying_side = HumanoidRobot::other_side(support.footsteps[0].side);

      // Computing the intermediary flying foot inflection target
      auto T_world_startTarget = trajectory.supports[step - 1].frame(flying_side);
      auto T_world_flyingTarget = trajectory.supports[step + 1].frame(flying_side);

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

WalkPatternGenerator::Trajectory WalkPatternGenerator::plan()
{
  WalkPatternGenerator::Trajectory trajectory;

  // Planning the supports followed by the walk
  trajectory.com_height = parameters.walk_com_height;
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;
  auto footsteps = footsteps_planner.plan(robot.flying_side, flatten_on_floor(robot.get_T_world_left()),
                                          flatten_on_floor(robot.get_T_world_right()));
  bool double_supports = parameters.double_support_duration / parameters.dt >= 1;
  trajectory.supports = footsteps_planner.make_supports(footsteps, true, double_supports, true);

  // Planning the center of mass trajectory
  planCoM(trajectory);

  // Planning the footsteps trajectories
  planFeetTrajectories(trajectory);

  return trajectory;
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::replan(WalkPatternGenerator::Trajectory& previous_trajectory,
                                                              double elapsed_time)
{
  WalkPatternGenerator::Trajectory trajectory;

  // Update the supports followed by the walk
  trajectory.com_height = parameters.walk_com_height;
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;

  auto T_world_left =
      flatten_on_floor(previous_trajectory.get_last_footstep_frame(HumanoidRobot::Side::Left, elapsed_time));
  auto T_world_right =
      flatten_on_floor(previous_trajectory.get_last_footstep_frame(HumanoidRobot::Side::Right, elapsed_time));

  auto footsteps = footsteps_planner.plan(robot.flying_side, T_world_left, T_world_right);
  bool double_supports = parameters.double_support_duration / parameters.dt >= 1;
  trajectory.supports = footsteps_planner.make_supports(footsteps, double_supports, double_supports, true);

  // Planning the center of mass trajectory
  planCoM(trajectory, previous_trajectory.com.vel(elapsed_time), previous_trajectory.com.acc(elapsed_time));

  // Planning the footsteps trajectories
  planFeetTrajectories(trajectory);

  return trajectory;
}

std::vector<FootstepsPlanner::Support>
WalkPatternGenerator::planSupportsKick(WalkPatternGenerator::Trajectory trajectory, HumanoidRobot::Side kicking_side,
                                       Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right)
{
  FootstepsPlanner::Footstep first_footstep(parameters.foot_width, parameters.foot_length);
  first_footstep.side = kicking_side;
  first_footstep.frame = kicking_side == HumanoidRobot::Side::Left ? T_world_left : T_world_right;

  FootstepsPlanner::Footstep second_footstep(parameters.foot_width, parameters.foot_length);
  second_footstep.side = HumanoidRobot::other_side(kicking_side);
  second_footstep.frame = kicking_side == HumanoidRobot::Side::Left ? T_world_right : T_world_left;
  second_footstep.support_polygon();

  FootstepsPlanner::Footstep third_footstep(parameters.foot_width, parameters.foot_length);
  third_footstep.side = kicking_side;
  third_footstep.frame = kicking_side == HumanoidRobot::Side::Left ? T_world_right : T_world_left;
  third_footstep.frame.translation().y() +=
      kicking_side == HumanoidRobot::Side::Left ? parameters.feet_spacing : -parameters.feet_spacing;

  std::vector<FootstepsPlanner::Support> supports;

  // First support
  FootstepsPlanner::Support support;
  support.start_end = true;
  support.footsteps = { first_footstep, second_footstep };
  supports.push_back(support);

  // Second support
  support.start_end = false;
  support.footsteps = { second_footstep };
  support.polygon = second_footstep.polygon;
  supports.push_back(support);

  // End support
  support.start_end = true;
  support.footsteps = { second_footstep, third_footstep };
  supports.push_back(support);

  return supports;
}

void WalkPatternGenerator::planCoMKick(Trajectory& trajectory, Eigen::Vector2d initial_vel, Eigen::Vector2d initial_acc)
{
  // Computing how many steps are required
  int ssp_steps = std::round(parameters.single_support_duration / parameters.dt);
  int dsp_steps = std::round(parameters.double_support_duration / parameters.dt);
  int kick_steps = std::round(parameters.kick_duration / parameters.dt);

  double total_steps = 3 * ssp_steps + kick_steps;
  trajectory.jerk_planner_steps = total_steps;

  // Creating the planner
  auto com_world = robot.com_world();
  auto target = trajectory.supports[1].frame().translation();
  JerkPlanner planner(total_steps, Eigen::Vector2d(com_world.x(), com_world.y()), initial_vel, initial_acc,
                      parameters.dt, parameters.omega());

  planner.add_equality_constraint(ssp_steps / 2, Eigen::Vector2d(target.x(), target.y()), JerkPlanner::ZMP);

  // Constraint the ZMP to be in the support polygon during the kick
  for (int i = ssp_steps / 2 + 1; i < total_steps - ssp_steps / 2; i++)
  {
    planner.add_polygon_constraint(i, trajectory.supports[1].polygon, JerkPlanner::ZMP, 0.02);
  }

  planner.add_equality_constraint(total_steps - ssp_steps / 2 + 1, Eigen::Vector2d(target.x(), target.y()),
                                  JerkPlanner::ZMP);

  // The CoM has to be stopped between the 2 feet at the end
  target = trajectory.supports[2].frame().translation();
  planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(target.x(), target.y()), JerkPlanner::Position);
  planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(0., 0.), JerkPlanner::Velocity);
  planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(0., 0.), JerkPlanner::Acceleration);

  trajectory.com = planner.plan();
}

void WalkPatternGenerator::planCoMOneFoot(Trajectory& trajectory, Eigen::Vector2d initial_vel,
                                          Eigen::Vector2d initial_acc)
{
  // Computing how many steps are required
  int ssp_steps = std::round(parameters.single_support_duration / parameters.dt);

  double total_steps = 2 * ssp_steps;
  trajectory.jerk_planner_steps = total_steps;

  // Creating the planner
  auto com_world = robot.com_world();
  auto target = trajectory.supports[1].frame().translation();
  JerkPlanner planner(total_steps, Eigen::Vector2d(com_world.x(), com_world.y()), initial_vel, initial_acc,
                      parameters.dt, parameters.omega());

  planner.add_equality_constraint(ssp_steps / 2, Eigen::Vector2d(target.x(), target.y()), JerkPlanner::ZMP);

  // Constraint the ZMP to be in the support polygon during the kick
  for (int i = ssp_steps / 2 + 1; i < total_steps - 1; i++)
  {
    planner.add_polygon_constraint(i, trajectory.supports[1].polygon, JerkPlanner::ZMP, 0.02);
  }

  // CoM constraints
  planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(target.x(), target.y()), JerkPlanner::Position);
  planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(0., 0.), JerkPlanner::Velocity);
  planner.add_equality_constraint(total_steps - 1, Eigen::Vector2d(0., 0.), JerkPlanner::Acceleration);

  trajectory.com = planner.plan();
}

void WalkPatternGenerator::planFeetKick(Trajectory& trajectory, HumanoidRobot::Side kicking_side,
                                        Eigen::Affine3d T_world_target)
{
  double t = 0.0;

  // Initial support
  _addSupports(trajectory, t, trajectory.supports[0]);
  trajectory.trunk_yaw.addPoint(0., frame_yaw(trajectory.supports[0].frame().rotation()), 0);

  // Trajectory to move the CoM above the foot use as support during the kick
  TrajectoryPart part;
  part.support = trajectory.supports[0];
  part.t_start = t;
  t += parameters.single_support_duration;
  part.t_end = t;

  _addSupports(trajectory, t, trajectory.supports[0]);
  trajectory.trunk_yaw.addPoint(t, frame_yaw(trajectory.supports[0].frame().rotation()), 0);
  trajectory.parts.push_back(part);

  // Trajectory to go to the targeted flying position
  part.support = trajectory.supports[1];
  part.t_start = t;
  part.swing_trajectory.t_start = t;
  t += parameters.single_support_duration;
  part.t_end = t;
  part.swing_trajectory.t_end = t;

  part.swing_trajectory.a = Eigen::Vector3d::Zero();
  part.swing_trajectory.b = Eigen::Vector3d::Zero();
  part.swing_trajectory.c = T_world_target.translation() - trajectory.supports[0].footsteps[0].frame.translation();
  part.swing_trajectory.d = trajectory.supports[0].footsteps[0].frame.translation();

  trajectory.yaw(kicking_side).addPoint(t, frame_yaw(T_world_target.rotation()), 0);
  _addSupports(trajectory, t, trajectory.supports[1]);
  trajectory.trunk_yaw.addPoint(t, frame_yaw(trajectory.supports[1].frame().rotation()), 0);
  trajectory.parts.push_back(part);

  // Trajectory of the kick
  part.support = trajectory.supports[1];
  part.t_start = t;
  part.swing_trajectory.t_start = t;
  t += parameters.kick_duration;
  part.t_end = t;
  part.swing_trajectory.t_end = t;

  auto T_world_kick_end = T_world_target;
  T_world_kick_end.translation().x() += 0.2;
  part.swing_trajectory.a = Eigen::Vector3d::Zero();
  part.swing_trajectory.b = T_world_kick_end.translation() - T_world_target.translation();
  part.swing_trajectory.c = Eigen::Vector3d::Zero();
  part.swing_trajectory.d = T_world_target.translation();
  trajectory.parts.push_back(part);

  // Trajectory to go back to the ground
  part.support = trajectory.supports[1];
  part.t_start = t;
  part.swing_trajectory.t_start = t;
  t += parameters.single_support_duration;
  part.t_end = t;
  part.swing_trajectory.t_end = t;

  auto T_world_end = trajectory.supports[2].frame(kicking_side);
  part.swing_trajectory.a = Eigen::Vector3d::Zero();
  part.swing_trajectory.b = Eigen::Vector3d::Zero();
  part.swing_trajectory.c = T_world_end.translation() - T_world_kick_end.translation();
  part.swing_trajectory.d = T_world_kick_end.translation();

  _addSupports(trajectory, t, trajectory.supports[2]);
  trajectory.trunk_yaw.addPoint(t, frame_yaw(trajectory.supports[2].frame().rotation()), 0);
  trajectory.parts.push_back(part);

  trajectory.duration = t;
}

void WalkPatternGenerator::planFeetOneFoot(Trajectory& trajectory, HumanoidRobot::Side kicking_side,
                                           Eigen::Affine3d T_world_target)
{
  double t = 0.;

  // Initial support
  _addSupports(trajectory, t, trajectory.supports[0]);
  trajectory.trunk_yaw.addPoint(0, frame_yaw(trajectory.supports[0].frame().rotation()), 0);

  // Trajectory to move the CoM above the foot use as support during the kick
  TrajectoryPart part;
  part.support = trajectory.supports[0];
  part.t_start = t;
  t += parameters.single_support_duration;
  part.t_end = t;

  _addSupports(trajectory, t, trajectory.supports[0]);
  trajectory.trunk_yaw.addPoint(t, frame_yaw(trajectory.supports[0].frame().rotation()), 0);
  trajectory.parts.push_back(part);

  // Trajectory to go to the targeted flying position
  part.support = trajectory.supports[1];
  part.t_start = t;
  part.swing_trajectory.t_start = t;
  t += parameters.single_support_duration;
  part.t_end = t;
  part.swing_trajectory.t_end = t;

  part.swing_trajectory.a = Eigen::Vector3d::Zero();
  part.swing_trajectory.b = Eigen::Vector3d::Zero();
  part.swing_trajectory.c = T_world_target.translation() - trajectory.supports[0].footsteps[0].frame.translation();
  part.swing_trajectory.d = trajectory.supports[0].footsteps[0].frame.translation();

  trajectory.yaw(kicking_side).addPoint(t, frame_yaw(T_world_target.rotation()), 0);
  _addSupports(trajectory, t, trajectory.supports[1]);
  trajectory.trunk_yaw.addPoint(t, frame_yaw(trajectory.supports[1].frame().rotation()), 0);
  trajectory.parts.push_back(part);

  trajectory.duration = t;
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::plan_kick(WalkPatternGenerator::Trajectory& previous_trajectory,
                                                                 double elapsed_time, HumanoidRobot::Side kicking_side,
                                                                 Eigen::Affine3d T_world_target)
{
  WalkPatternGenerator::Trajectory trajectory;

  // Update the supports followed by the walk
  trajectory.com_height = parameters.walk_com_height;
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;

  auto T_world_left =
      flatten_on_floor(previous_trajectory.get_last_footstep_frame(HumanoidRobot::Side::Left, elapsed_time));
  auto T_world_right =
      flatten_on_floor(previous_trajectory.get_last_footstep_frame(HumanoidRobot::Side::Right, elapsed_time));

  // Planning the supports
  trajectory.supports = planSupportsKick(trajectory, kicking_side, T_world_left, T_world_right);

  // Planning the center of mass trajectory
  planCoMKick(trajectory, previous_trajectory.com.vel(elapsed_time), previous_trajectory.com.acc(elapsed_time));

  // Planning the feet trajectories
  planFeetKick(trajectory, kicking_side, T_world_target);

  return trajectory;
}

WalkPatternGenerator::Trajectory
WalkPatternGenerator::plan_one_foot_balance(WalkPatternGenerator::Trajectory& previous_trajectory, double elapsed_time,
                                            HumanoidRobot::Side flying_side, Eigen::Affine3d T_world_target)
{
  WalkPatternGenerator::Trajectory trajectory;

  // Update the supports followed by the walk
  trajectory.com_height = parameters.walk_com_height;
  trajectory.trunk_pitch = parameters.walk_trunk_pitch;

  auto T_world_left =
      flatten_on_floor(previous_trajectory.get_last_footstep_frame(HumanoidRobot::Side::Left, elapsed_time));
  auto T_world_right =
      flatten_on_floor(previous_trajectory.get_last_footstep_frame(HumanoidRobot::Side::Right, elapsed_time));

  // Planning the supports
  auto supports = planSupportsKick(trajectory, flying_side, T_world_left, T_world_right);
  trajectory.supports = { supports[0], supports[1] };

  // Planning the center of mass trajectory
  planCoMOneFoot(trajectory, previous_trajectory.com.vel(elapsed_time), previous_trajectory.com.acc(elapsed_time));

  // Planning the feet trajectories
  planFeetOneFoot(trajectory, flying_side, T_world_target);

  return trajectory;
}
}  // namespace placo