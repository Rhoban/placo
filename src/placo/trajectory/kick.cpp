#include "placo/trajectory/kick.h"
#include "placo/utils.h"

namespace placo
{
Eigen::Vector3d Kick::KickTrajectory::pos(double t)
{
  return foot_trajectory.pos(t);
}

Eigen::Vector3d Kick::KickTrajectory::vel(double t)
{
  return foot_trajectory.vel(t);
}

Kick::KickTrajectory Kick::make_trajectory(HumanoidRobot::Side kicking_side, double t_start, double t_end,
                                           Eigen::Vector3d start, Eigen::Vector3d target, Eigen::Vector3d neutral)
{
  KickTrajectory trajectory;

  double duration = t_end - t_start;
  double t_up = duration * trajectory.ratio_up;
  double t_down = duration * (trajectory.ratio_up + trajectory.ratio_delay);

  Eigen::Vector3d flying_position = neutral;
  flying_position.z() = trajectory.kicking_foot_height;

  trajectory.foot_trajectory.add_point(0, start, Eigen::Vector3d::Zero());
  trajectory.foot_trajectory.add_point(t_up, flying_position, Eigen::Vector3d::Zero());
  trajectory.foot_trajectory.add_point(t_down, flying_position, Eigen::Vector3d::Zero());
  trajectory.foot_trajectory.add_point(duration, target, Eigen::Vector3d::Zero());

  return trajectory;
}

// trajectory.foot_trajectory.add_point(t_init + t_pre_delay, T_world_left.translation(), Eigen::Vector3d::Zero());
// right_foot_trajectory.add_point(0, T_world_right.translation(), Eigen::Vector3d::Zero());
// right_foot_trajectory.add_point(t_init + t_pre_delay, T_world_right.translation(), Eigen::Vector3d::Zero());

// if (support_side == HumanoidRobot::Left)
// {
//   support_frame = T_world_left;
//   single_support.footsteps = { left_footstep };

//   left_foot_trajectory.add_point(t_init + t_pre_delay + t_up, T_world_left.translation(), Eigen::Vector3d::Zero());
//   left_foot_trajectory.add_point(t_init + t_pre_delay + t_up + t_post_delay, T_world_left.translation(),
//                                  Eigen::Vector3d::Zero());

//   Eigen::Vector3d flying_position = T_world_right * Eigen::Vector3d(0., 0., kicking_foot_height);
//   right_foot_trajectory.add_point(t_init + t_pre_delay + t_up, flying_position, Eigen::Vector3d::Zero());
//   right_foot_trajectory.add_point(t_init + t_pre_delay + t_up + t_post_delay, flying_position,
//                                   Eigen::Vector3d::Zero());
// }
// else
// {
//   support_frame = T_world_right;
//   single_support.footsteps = { right_footstep };

//   right_foot_trajectory.add_point(t_init + t_pre_delay + t_up, T_world_right.translation(), Eigen::Vector3d::Zero());
//   right_foot_trajectory.add_point(t_init + t_pre_delay + t_up + t_post_delay, T_world_right.translation(),
//                                   Eigen::Vector3d::Zero());

//   Eigen::Vector3d flying_position = T_world_left * Eigen::Vector3d(0., 0., kicking_foot_height);
//   left_foot_trajectory.add_point(t_init + t_pre_delay + t_up, flying_position, Eigen::Vector3d::Zero());
//   left_foot_trajectory.add_point(t_init + t_pre_delay + t_up + t_post_delay, flying_position,
//                                  Eigen::Vector3d::Zero());

// // CoM jerk_trajectory
// int t_init_ts = std::round(t_init / parameters.dt());
// int t_pre_delay_ts = std::round(t_pre_delay / parameters.dt());
// int t_up_ts = std::round(t_up / parameters.dt());
// int t_post_delay_ts = std::round(t_post_delay / parameters.dt());
// int nb_timesteps = t_init_ts + t_pre_delay_ts + t_up_ts + t_post_delay_ts;

// Eigen::Vector3d com_world = robot.com_world();
// Problem problem;
// LIPM lipm(problem, nb_timesteps, parameters.omega(), parameters.dt(), Eigen::Vector2d(com_world[0], com_world[1]),
//           Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());

// for (int timestep = 0; timestep < t_init_ts; timestep++)
// {
//   problem.add_constraints(
//       PolygonConstraint::in_polygon_xy(lipm.zmp(timestep), double_support.support_polygon(),
//       parameters.zmp_margin));
// }

// // CoM target
// Eigen::Affine3d target = single_support.frame();
// double offset = com_support_offset > 0.03 ? 0.03 : com_support_offset;
// target.translation() = target * (Eigen::Vector3d::UnitY() * offset);

// problem.add_constraint(lipm.pos(t_init_ts) == Eigen::Vector2d(target.translation().x(), target.translation().y()));
// problem.add_constraint(lipm.vel(t_init_ts) == Eigen::Vector2d(0., 0.));
// problem.add_constraint(lipm.acc(t_init_ts) == Eigen::Vector2d(0., 0.));

// problem.solve();
// com_trajectory = lipm.get_trajectory();

// // CoM height
// com_height.add_point(0, parameters.walk_com_height, 0);
// com_height.add_point(t_init, kicking_com_height, 0);
// com_height.add_point(t_init + t_pre_delay + t_up + t_post_delay, kicking_com_height, 0);

// duration = t_init + t_pre_delay + t_up + t_post_delay;

}  // namespace placo