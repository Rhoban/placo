#include "placo/planning/kick.h"
#include "placo/utils.h"
#include "placo/problem/polygon_constraint.h"

namespace placo
{
Kick::Kick(HumanoidRobot& robot, HumanoidParameters& parameters) : robot(robot), parameters(parameters)
{
}

void Kick::one_foot_balance(FootstepsPlanner& planner, HumanoidRobot::Side support_side_)
{
  Eigen::Affine3d T_world_left = placo::flatten_on_floor(robot.get_T_world_left());
  Eigen::Affine3d T_world_right = placo::flatten_on_floor(robot.get_T_world_right());
  auto left_footstep = planner.create_footstep(HumanoidRobot::Left, T_world_left);
  auto right_footstep = planner.create_footstep(HumanoidRobot::Right, T_world_right);

  FootstepsPlanner::Support double_support;
  double_support.footsteps = { left_footstep, right_footstep };
  FootstepsPlanner::Support single_support;
  support_side = support_side_;

  // Feet trajectories
  left_foot_trajectory.add_point(0, T_world_left.translation(), Eigen::Vector3d::Zero());
  left_foot_trajectory.add_point(t_init + t_delay, T_world_left.translation(), Eigen::Vector3d::Zero());
  right_foot_trajectory.add_point(0, T_world_right.translation(), Eigen::Vector3d::Zero());
  right_foot_trajectory.add_point(t_init + t_delay, T_world_right.translation(), Eigen::Vector3d::Zero());

  if (support_side == HumanoidRobot::Left)
  {
    support_frame = T_world_left;
    single_support.footsteps = { left_footstep };

    left_foot_trajectory.add_point(t_init + t_delay + t_up, T_world_left.translation(), Eigen::Vector3d::Zero());
    Eigen::Vector3d flying_position = T_world_right * Eigen::Vector3d(0., 0., kick_foot_height);
    right_foot_trajectory.add_point(t_init + t_delay + t_up, flying_position, Eigen::Vector3d::Zero());
  }
  else
  {
    support_frame = T_world_right;
    single_support.footsteps = { right_footstep };

    right_foot_trajectory.add_point(t_init + t_delay + t_up, T_world_right.translation(), Eigen::Vector3d::Zero());
    Eigen::Vector3d flying_position = T_world_left * Eigen::Vector3d(0., 0., parameters.walk_foot_height);
    left_foot_trajectory.add_point(t_init + t_delay + t_up, flying_position, Eigen::Vector3d::Zero());
  }

  // CoM jerk_trajectory
  int t_init_ts = std::round(t_init / parameters.dt());
  int t_delay_ts = std::round(t_delay / parameters.dt());
  int t_up_ts = std::round(t_up / parameters.dt());
  int nb_timesteps = t_init_ts + t_delay_ts + t_up_ts;

  Eigen::Vector3d com_world = robot.com_world();
  Problem problem;
  LIPM lipm(problem, nb_timesteps, parameters.omega(), parameters.dt(), Eigen::Vector2d(com_world[0], com_world[1]),
            Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());

  for (int timestep = 0; timestep < t_init_ts; timestep++)
  {
    problem.add_constraints(
        PolygonConstraint::in_polygon_xy(lipm.zmp(timestep), double_support.support_polygon(), parameters.zmp_margin));
  }

  problem.add_constraint(lipm.pos(t_init_ts) == Eigen::Vector2d(single_support.frame().translation().x(),
                                                                single_support.frame().translation().y()));
  problem.add_constraint(lipm.vel(t_init_ts) == Eigen::Vector2d(0., 0.));
  problem.add_constraint(lipm.acc(t_init_ts) == Eigen::Vector2d(0., 0.));

  problem.solve();
  com_trajectory = lipm.get_trajectory();

  // CoM height
  com_height.add_point(0, parameters.walk_com_height, 0);
  com_height.add_point(t_init, kick_com_height, 0);
  com_height.add_point(t_init + t_delay + t_up, kick_com_height, 0);
  duration = t_init + t_delay + t_up;
}

Eigen::Affine3d Kick::get_T_world_left(double t)
{
  Eigen::Affine3d T_world_left = robot.get_T_world_left();
  T_world_left.translation() = left_foot_trajectory.pos(t);

  return T_world_left;
}

Eigen::Affine3d Kick::get_T_world_right(double t)
{
  Eigen::Affine3d T_world_right = robot.get_T_world_right();
  T_world_right.translation() = right_foot_trajectory.pos(t);

  return T_world_right;
}

Eigen::Vector3d Kick::get_com_world(double t)
{
  return Eigen::Vector3d(com_trajectory.pos(t)[0], com_trajectory.pos(t)[1], com_height.pos(t));
}

bool Kick::support_is_both(double t)
{
  return false;
  // return t < (t_init + t_delay);
}
}  // namespace placo