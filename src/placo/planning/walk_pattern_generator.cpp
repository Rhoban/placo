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
  // Computing how many steps are required
  int ssp_steps = std::round(parameters.single_support_duration / parameters.dt);
  int dsp_steps = std::round(parameters.double_support_duration / parameters.dt);
  int se_dsp_steps = std::round(parameters.startend_double_support_duration / parameters.dt);
  int total_steps = 0;

  for (auto& support : trajectory.footsteps)
  {
    if (support.footsteps.size() == 1)
    {
      total_steps += ssp_steps;
    }
    else
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
  int steps = 0;
  for (auto& support : trajectory.footsteps)
  {
    if (support.footsteps.size() == 1)
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
          planner.add_equality_constraint(steps + k, Eigen::Vector2d(target.x(), target.y()), JerkPlanner::ZMP);
        }
      }

      steps += ssp_steps;
    }
    else
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

static Eigen::Affine3d _buildFrame(Eigen::Vector3d position, Eigen::Vector3d velocity, double orientation, double tilt)
{
  Eigen::Affine3d frame = Eigen::Affine3d::Identity();

  frame.translation() = position;
  frame.linear() = Eigen::AngleAxisd(orientation, Eigen::Vector3d::UnitZ()).matrix();
  if (velocity.norm() > 1e-3)
  {
    frame.linear() = frame.linear() *
                     Eigen::AngleAxisd(-tilt, Eigen::Vector3d(-velocity.y(), velocity.x(), 0.).normalized()).matrix();
  }

  return frame;
}

static SwingFoot& _findSwingFoot(std::vector<SwingFoot>& swings, double t)
{
  for (auto& swing : swings)
  {
    if (t >= swing.t_start && t <= swing.t_start + swing.duration)
    {
      return swing;
    }
  }

  return swings[swings.size() - 1];
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_left(double t)
{
  SwingFoot& swing = _findSwingFoot(left_foot, t);

  return _buildFrame(swing.pos(t), swing.vel(t), left_foot_yaw.get(t), left_foot_tilt.get(t));
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_right(double t)
{
  SwingFoot& swing = _findSwingFoot(right_foot, t);

  return _buildFrame(swing.pos(t), swing.vel(t), right_foot_yaw.get(t), right_foot_tilt.get(t));
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

rhoban_utils::PolySpline& WalkPatternGenerator::Trajectory::tilt(HumanoidRobot::Side side)
{
  if (side == HumanoidRobot::Left)
  {
    return left_foot_tilt;
  }
  else
  {
    return right_foot_tilt;
  }
}

std::vector<SwingFoot>& WalkPatternGenerator::Trajectory::swing_foot(HumanoidRobot::Side side)
{
  if (side == HumanoidRobot::Left)
  {
    return left_foot;
  }
  else
  {
    return right_foot;
  }
}

static void _addSupports(WalkPatternGenerator::Trajectory& trajectory, double t, FootstepsPlanner::Support& support,
                         double duration)
{
  for (auto footstep : support.footsteps)
  {
    auto T_world_foot = footstep.frame;
    trajectory.yaw(footstep.side).addPoint(t, frame_yaw(T_world_foot.rotation()), 0);
    trajectory.tilt(footstep.side).addPoint(t, 0, 0);

    if (duration > 0.)
    {
      auto pos = flatten_on_floor(T_world_foot).translation();
      SwingFoot no_swing(duration, 0., pos, pos);
      no_swing.t_start = t - duration;
      trajectory.swing_foot(footstep.side).push_back(no_swing);
    }
  }
}

void WalkPatternGenerator::planFeetTrajctories(Trajectory& trajectory)
{
  double t = 0.0;

  // First, adds the initial position to the trajectory
  _addSupports(trajectory, 0., trajectory.footsteps[0], 0.);
  trajectory.trunk_yaw.addPoint(0, frame_yaw(trajectory.footsteps[0].frame().rotation()), 0);

  for (int step = 0; step < trajectory.footsteps.size(); step++)
  {
    auto& support = trajectory.footsteps[step];

    if (support.footsteps.size() == 1)
    {
      // Single support, add the flying trajectory
      auto flying_side = HumanoidRobot::other_side(support.footsteps[0].side);

      // Computing the intermediary flying foot inflection target
      auto T_world_startTarget = trajectory.footsteps[step - 1].frame(flying_side);
      auto T_world_flyingTarget = trajectory.footsteps[step + 1].frame(flying_side);

      trajectory.tilt(flying_side).addPoint(t + parameters.single_support_duration / 4, -parameters.walk_foot_tilt, 0.);
      trajectory.tilt(flying_side)
          .addPoint(t + parameters.single_support_duration * 3 / 4, parameters.walk_foot_tilt, 0.);

      t += parameters.single_support_duration;

      // Flying foot reaching its position
      SwingFoot swing(parameters.single_support_duration, parameters.walk_foot_height,
                      T_world_startTarget.translation(), T_world_flyingTarget.translation());
      swing.t_start = t - parameters.single_support_duration;

      trajectory.swing_foot(flying_side).push_back(swing);
      trajectory.yaw(flying_side).addPoint(t, frame_yaw(T_world_flyingTarget.rotation()), 0);
      trajectory.trunk_yaw.addPoint(t, frame_yaw(T_world_flyingTarget.rotation()), 0);

      trajectory.tilt(flying_side).addPoint(t, 0., 0);

      // Support foot remaining steady
      _addSupports(trajectory, t, support, parameters.single_support_duration);
    }
    else
    {
      // Double support, adding the support foot at the begining and at the end of the trajectory
      if (support.start_end)
      {
        t += parameters.startend_double_support_duration;
        _addSupports(trajectory, t, support, parameters.startend_double_support_duration);
      }
      else
      {
        t += parameters.double_support_duration;
        _addSupports(trajectory, t, support, parameters.double_support_duration);
      }
      trajectory.trunk_yaw.addPoint(t, frame_yaw(support.frame().rotation()), 0);
    }
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
  planFeetTrajctories(trajectory);

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
  planFeetTrajctories(trajectory);

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
}  // namespace placo