#include "placo/planning/walk_pattern_generator.h"
#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/utils.h"

namespace placo
{
WalkPatternGenerator::WalkPatternGenerator(HumanoidRobot& robot) : robot(robot)
{
}

void WalkPatternGenerator::planFootsteps(Trajectory& trajectory, Eigen::Affine3d T_world_left,
                                         Eigen::Affine3d T_world_right)
{
  // Retrieving current foot frames
  auto T_world_leftCurrent = flatten_on_floor(robot.get_T_world_left());
  auto T_world_rightCurrent = flatten_on_floor(robot.get_T_world_right());

  // Creates the planner and run the planning
  FootstepsPlannerNaive planner(robot.support_side, T_world_leftCurrent, T_world_rightCurrent);
  auto footsteps = planner.plan(T_world_left, T_world_right);

  trajectory.footsteps = planner.make_double_supports(footsteps, true, false, true);
}

void WalkPatternGenerator::planCoM(Trajectory& trajectory)
{
  // Computing how many steps are required
  int ssp_steps = std::round(parameters.single_support_duration / parameters.dt);
  int dsp_steps = std::round(parameters.double_support_duration / parameters.dt);
  int total_steps = 0;

  for (auto& support : trajectory.footsteps)
  {
    if (support.footsteps.size() == 1)
    {
      total_steps += ssp_steps;
    }
    else
    {
      total_steps += dsp_steps;
    }

    if (total_steps >= parameters.maximum_steps)
    {
      break;
    }
  }

  // Creating the planner
  JerkPlanner planner(total_steps, Eigen::Vector2d(0., 0.), Eigen::Vector2d(0., 0.), Eigen::Vector2d(0., 0.),
                      parameters.dt, parameters.omega());

  // Adding constraints
  int steps = 0;
  for (auto& support : trajectory.footsteps)
  {
    if (support.footsteps.size() == 1)
    {
      // Adding a constraint at the begining of the stem
      planner.add_polygon_constraint(steps, support.support_polygon(), JerkPlanner::ZMP, parameters.zmp_margin);

      steps += ssp_steps;
    }
    else
    {
      steps += dsp_steps;
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

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_left(double t)
{
  return _buildFrame(left_foot.get(t), left_foot_yaw.get(t));
}

Eigen::Affine3d WalkPatternGenerator::Trajectory::get_T_world_right(double t)
{
  return _buildFrame(right_foot.get(t), right_foot_yaw.get(t));
}

Eigen::Vector3d WalkPatternGenerator::Trajectory::get_CoM_world(double t)
{
  auto pos = com.pos(t);

  return Eigen::Vector3d(pos.x(), pos.y(), com_height);
}

Eigen::Matrix3d WalkPatternGenerator::Trajectory::get_R_world_trunk(double t)
{
  return Eigen::AngleAxisd(trunk_yaw.get(t), Eigen::Vector3d::UnitZ()).matrix();
}

rhoban_utils::PolySpline3D& WalkPatternGenerator::Trajectory::position(HumanoidRobot::Side side)
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
    std::cout << T_world_foot.translation().z() << std::endl;
    trajectory.position(footstep.side).addPoint(t, T_world_foot.translation(), Eigen::Vector3d::Zero());
    trajectory.yaw(footstep.side).addPoint(t, frame_yaw(T_world_foot.rotation()), 0);
  }
}

void WalkPatternGenerator::planFeetTrajctories(Trajectory& trajectory)
{
  double t = 0.0;

  // First, adds the initial position to the trajectory
  _addSupports(trajectory, 0., trajectory.footsteps[0]);
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

      Eigen::Vector3d delta_step = T_world_flyingTarget.translation() - T_world_startTarget.translation();
      Eigen::Vector3d flying_mid = T_world_startTarget.translation() + (delta_step) / 2.;

      flying_mid.z() = parameters.walk_foot_height;

      // XXX: The speed at inflection point could be parametrized
      Eigen::Vector3d flying_mid_delta = delta_step / parameters.single_support_duration;

      trajectory.position(flying_side)
          .addPoint(t + parameters.single_support_duration / 2., flying_mid, flying_mid_delta);

      t += parameters.single_support_duration;

      // Flying foot reaching its position
      trajectory.position(flying_side).addPoint(t, T_world_flyingTarget.translation(), Eigen::Vector3d::Zero());
      trajectory.yaw(flying_side).addPoint(t, frame_yaw(T_world_flyingTarget.rotation()), 0);
      trajectory.trunk_yaw.addPoint(0, frame_yaw(T_world_flyingTarget.rotation()), 0);

      // Support foot remaining steady
      _addSupports(trajectory, t, support);
    }
    else
    {
      // Double support, adding the support foot at the begining and at the end of the trajectory
      t += parameters.double_support_duration;
      _addSupports(trajectory, t, support);
      trajectory.trunk_yaw.addPoint(0, frame_yaw(support.frame().rotation()), 0);
    }
  }

  trajectory.duration = t;
}

WalkPatternGenerator::Trajectory WalkPatternGenerator::plan(Eigen::Affine3d T_world_left, Eigen::Affine3d T_world_right)
{
  Trajectory trajectory;
  trajectory.com_height = parameters.walk_com_height;

  // Planning the footsteps to take
  planFootsteps(trajectory, T_world_left, T_world_right);

  // Planning the center of mass trajectory
  planCoM(trajectory);

  // Planning the footsteps trajectories
  planFeetTrajctories(trajectory);

  return trajectory;
}
}  // namespace placo