#include "placo/planning/walk_pattern_generator.h"
#include "placo/footsteps/footsteps_planner_naive.h"
#include "placo/footsteps/footsteps_planner_repetitive.h"
#include "placo/utils.h"

namespace placo
{
WalkPatternGenerator::WalkPatternGenerator(HumanoidRobot& robot, KinematicsSolver& solver, FootstepsPlanner& planner)
  : robot(robot), solver(solver), footsteps_planner(planner)
{
  init_default_solver_tasks();
  time = 0.;
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

  // Search in the previous segment of trajectory
  part = _findPart(parts, part.t_start - 1e-5);
  for (auto footstep : part.support.footsteps)
  {
    if (footstep.side == side)
    {
      return footstep.frame;
    }
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

void WalkPatternGenerator::planCoM(Eigen::Vector2d initial_vel, Eigen::Vector2d initial_acc)
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
        if (k == (ssp_steps / 2))
        {
          auto target = support.frame().translation();
          planner.add_equality_constraint(steps + k, Eigen::Vector2d(target.x(), target.y()), JerkPlanner::ZMP);
        }
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

void WalkPatternGenerator::planFeetTrajectories()
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

void WalkPatternGenerator::init_default_solver_tasks()
{
  solver.mask_dof("head_pitch");
  solver.mask_dof("head_yaw");
  solver.mask_dof("left_elbow");
  solver.mask_dof("right_elbow");
  solver.mask_dof("left_shoulder_pitch");
  solver.mask_dof("right_shoulder_pitch");
  solver.mask_dof("left_shoulder_roll");
  solver.mask_dof("right_shoulder_roll");

  left_foot = solver.add_frame_task("left_foot", robot.get_T_world_left());
  left_foot.configure("left_foot", "soft", 1., 1.);

  right_foot = solver.add_frame_task("right_foot", robot.get_T_world_right());
  right_foot.configure("right_foot", "soft", 1., 1.);

  com_task.configure("com", "soft", 1.0);
  trunk_orientation_task.configure("trunk", "soft", 1.0);

  solver.add_regularization_task(1e-6);
  solver.solve(true);
  robot.update_kinematics();
}

void WalkPatternGenerator::update_trajectory()
{
  WalkPatternGenerator::Trajectory computed_trajectory;

  // Update the supports followed by the walk
  computed_trajectory.com_height = parameters.walk_com_height;
  computed_trajectory.trunk_pitch = parameters.walk_trunk_pitch;
  computed_trajectory.supports = footsteps_planner.supports;
  footsteps_planner.new_supports = false;

  // Planning the center of mass trajectory
  planCoM();

  // Planning the footsteps trajectories
  planFeetTrajectories();

  trajectory = computed_trajectory;
}

void WalkPatternGenerator::next(double elapsed_time)
{
  time += elapsed_time;

  if (footsteps_planner.new_supports)
  {
    update_trajectory();
    time = 0.;
  }

  left_foot.set_T_world_frame(trajectory.get_T_world_left(time));
  right_foot.set_T_world_frame(trajectory.get_T_world_right(time));
  com_task.target_world = trajectory.get_CoM_world(time);
  trunk_orientation_task.R_world_frame = trajectory.get_R_world_trunk(time);
  solver.solve(true);

  robot.update_kinematics();
  robot.update_support_side(trajectory.support_side(time));
}
}  // namespace placo