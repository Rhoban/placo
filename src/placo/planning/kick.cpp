#include "placo/planning/kick.h"
#include "placo/utils.h"

namespace placo
{
Kick::Kick(HumanoidRobot& robot, HumanoidParameters& parameters) : robot(robot), parameters(parameters)
{
}

void Kick::one_foot_balance(FootstepsPlanner& planner, HumanoidRobot::Side support_side)
{
  Eigen::Affine3d T_world_left = placo::flatten_on_floor(robot.get_T_world_left());
  Eigen::Affine3d T_world_right = placo::flatten_on_floor(robot.get_T_world_right());
  auto left_footstep = planner.create_footstep(HumanoidRobot::Left, T_world_left);
  auto right_footstep = planner.create_footstep(HumanoidRobot::Right, T_world_right);

  FootstepsPlanner::Support double_support;
  double_support.footsteps = { left_footstep, right_footstep };
  FootstepsPlanner::Support single_support;

  // Feet trajectories
  left_foot_trajectory.add_point(0, T_world_left.translation(), Eigen::Vector3d::Zero());
  left_foot_trajectory.add_point(t_init, T_world_left.translation(), Eigen::Vector3d::Zero());
  right_foot_trajectory.add_point(0, T_world_right.translation(), Eigen::Vector3d::Zero());
  right_foot_trajectory.add_point(t_init, T_world_right.translation(), Eigen::Vector3d::Zero());

  if (support_side == HumanoidRobot::Left)
  {
    single_support.footsteps = { left_footstep };

    left_foot_trajectory.add_point(t_init + t_up, T_world_left.translation(), Eigen::Vector3d::Zero());
    Eigen::Vector3d flying_position = T_world_right * Eigen::Vector3d(0., 0., parameters.walk_foot_height);
    right_foot_trajectory.add_point(t_init + t_up, flying_position, Eigen::Vector3d::Zero());
  }
  else
  {
    single_support.footsteps = { right_footstep };

    right_foot_trajectory.add_point(t_init + t_up, T_world_right.translation(), Eigen::Vector3d::Zero());
    Eigen::Vector3d flying_position = T_world_left * Eigen::Vector3d(0., 0., parameters.walk_foot_height);
    left_foot_trajectory.add_point(t_init + t_up, flying_position, Eigen::Vector3d::Zero());
  }

  // CoM jerk_trajectory
  int t_init_ts = std::round(t_init / parameters.dt());
  int t_up_ts = std::round(t_up / parameters.dt());
  int nb_timesteps = t_init_ts + t_up_ts;

  Eigen::Vector3d com_world = robot.com_world();
  // JerkPlanner jerk_planner(nb_timesteps, Eigen::Vector2d(com_world[0], com_world[1]), Eigen::Vector2d::Zero(),
  //                          Eigen::Vector2d::Zero(), parameters.dt(), parameters.omega());

  // for (int timestep = 0; timestep < t_init_ts; timestep++)
  // {
  //   jerk_planner.add_polygon_constraint(timestep, double_support.support_polygon(), JerkPlanner::ZMP,
  //                                       parameters.zmp_margin);
  // }

  // jerk_planner.add_equality_constraint(
  //     t_init_ts, Eigen::Vector2d(single_support.frame().translation().x(), single_support.frame().translation().y()),
  //     JerkPlanner::Position);
  // jerk_planner.add_equality_constraint(t_init_ts, Eigen::Vector2d(0., 0.), JerkPlanner::Velocity);
  // jerk_planner.add_equality_constraint(t_init_ts, Eigen::Vector2d(0., 0.), JerkPlanner::Acceleration);

  // com_trajectory = jerk_planner.plan();

  // CoM height
  com_height.add_point(0, parameters.walk_com_height, 0);
  com_height.add_point(t_init, kick_com_height, 0);
  com_height.add_point(t_init + t_up, kick_com_height, 0);
  duration = t_init + t_up;
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
  // return Eigen::Vector3d(com_trajectory.pos(t)[0], com_trajectory.pos(t)[1], com_height.pos(t));
}
}  // namespace placo