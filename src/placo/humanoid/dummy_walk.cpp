#include "placo/humanoid/dummy_walk.h"

namespace placo::humanoid
{
DummyWalk::DummyWalk(model::RobotWrapper& robot, humanoid::HumanoidParameters& parameters)
  : robot(robot), parameters(parameters), solver(robot), footsteps_planner(parameters)
{
  // Initializing solver
  solver.enable_velocity_limits(true);
  solver.dt = 0.1;

  left_foot_task = solver.add_frame_task("left_foot");
  left_foot_task.configure("left_foot", "soft", 1.0, 1.0);

  right_foot_task = solver.add_frame_task("right_foot");
  right_foot_task.configure("right_foot", "soft", 1.0, 1.0);

  trunk_task = solver.add_frame_task("trunk");
  trunk_task.configure("trunk", "soft", 1.0, 1.0);

  reset();
}

void DummyWalk::reset(bool support_left_)
{
  // Initializing lift trajectory
  lift_spline.clear();
  lift_spline.add_point(0, 0, 0);
  lift_spline.add_point(0.5 - parameters.walk_foot_rise_ratio / 2, parameters.walk_foot_height, 0);
  lift_spline.add_point(0.5 + parameters.walk_foot_rise_ratio / 2, parameters.walk_foot_height, 0);
  lift_spline.add_point(1, 0, 0);

  robot.reset();
  robot.update_kinematics();

  support_left = support_left_;

  T_world_left = translation(0, parameters.feet_spacing / 2, 0);
  T_world_right = translation(0, -parameters.feet_spacing / 2, 0);

  compute_next_support(0.0, 0.0, 0.0);
  update(0.0);
}

void DummyWalk::next_step(double dx, double dy, double dtheta)
{
  if (support_left)
  {
    T_world_right = T_world_next;
  }
  else
  {
    T_world_left = T_world_next;
  }

  support_left = !support_left;
  compute_next_support(dx, dy, dtheta);
}

void DummyWalk::update(double t)
{
  Eigen::Affine3d T_world_left_ = T_world_left;
  Eigen::Affine3d T_world_right_ = T_world_right;

  if (support_left)
  {
    T_world_right_ = tools::interpolate_frames(T_world_right, T_world_next, t);
    T_world_right_.translation().z() = lift_spline.pos(t);
  }
  else
  {
    T_world_left_ = tools::interpolate_frames(T_world_left, T_world_next, t);
    T_world_left_.translation().z() = lift_spline.pos(t);
  }

  Eigen::Affine3d T_world_mid = placo::tools::interpolate_frames(tools::flatten_on_floor(T_world_left_),
                                                                 tools::flatten_on_floor(T_world_right_), 0.5);
  Eigen::Affine3d T_world_trunk = T_world_mid * translation(trunk_x_offset, 0, parameters.walk_com_height) *
                                  Eigen::AngleAxisd(parameters.walk_trunk_pitch, Eigen::Vector3d::UnitY());

  left_foot_task.set_T_world_frame(T_world_left_);
  right_foot_task.set_T_world_frame(T_world_right_);
  trunk_task.set_T_world_frame(T_world_trunk);

  solve();
}

void DummyWalk::update_T_world_support(Eigen::Affine3d T_world_support)
{
  Eigen::Affine3d T_world_currentSupport = support_left ? T_world_left : T_world_right;
  Eigen::Affine3d T = tools::flatten_on_floor(T_world_support) * T_world_currentSupport.inverse();

  T_world_left = tools::flatten_on_floor(T * T_world_left);
  T_world_right = tools::flatten_on_floor(T * T_world_right);
  T_world_next = tools::flatten_on_floor(T * T_world_next);

  if (support_left)
  {
    robot.set_T_world_frame("left_foot", T_world_left);
  }
  else
  {
    robot.set_T_world_frame("right_foot", T_world_right);
  }
  robot.update_kinematics();
}

void DummyWalk::compute_next_support(double dx_, double dy_, double dtheta_)
{
  dx = dx_;
  dy = dy_;
  dtheta = dtheta_;

  footsteps_planner.configure(dx, dy, dtheta, 2);
  std::vector<FootstepsPlanner::Footstep> footsteps = footsteps_planner.plan(
      support_left ? HumanoidRobot::Side::Right : HumanoidRobot::Side::Left, T_world_left, T_world_right);

  T_world_next = footsteps[2].frame;
}

Eigen::Affine3d DummyWalk::translation(double x, double y, double z) const
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

void DummyWalk::solve()
{
  for (int k = 0; k < 4; k++)
  {
    robot.add_q_noise(1e-3);
    robot.update_kinematics();
    solver.solve(true);
  }
}
}  // namespace placo::humanoid