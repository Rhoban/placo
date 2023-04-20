#include "placo/planning/walk_tasks.h"
#include "placo/model/humanoid_robot.h"

namespace placo
{
void WalkTasks::initialize_tasks(KinematicsSolver* solver_)
{
  solver = solver_;
  HumanoidRobot* robot = dynamic_cast<HumanoidRobot*>(solver->robot);

  if (robot == nullptr)
  {
    throw std::runtime_error("WalkTasks should be used with an humanoid robot");
  }

  left_foot_task = solver->add_frame_task("left_foot", robot->get_T_world_left());
  left_foot_task.configure("left_foot", "soft", 1., 1.);

  right_foot_task = solver->add_frame_task("right_foot", robot->get_T_world_right());
  right_foot_task.configure("right_foot", "soft", 1., 1.);

  com_task = &solver->add_com_task(robot->com_world());
  com_task->configure("com", "soft", 1.);

  trunk_orientation_task = &solver->add_orientation_task("trunk", robot->get_T_world_trunk().rotation());
  trunk_orientation_task->configure("trunk", "soft", 1.);
}

void WalkTasks::update_tasks(WalkPatternGenerator::Trajectory& trajectory, double t)
{
  left_foot_task.set_T_world_frame(trajectory.get_T_world_left(t));
  right_foot_task.set_T_world_frame(trajectory.get_T_world_right(t));
  com_task->target_world = trajectory.get_p_world_CoM(t);
  trunk_orientation_task->R_world_frame = trajectory.get_R_world_trunk(t);
}

void WalkTasks::remove_tasks()
{
  if (solver != nullptr)
  {
    solver->remove_task(left_foot_task);
    solver->remove_task(right_foot_task);
    solver->remove_task(com_task);
    solver->remove_task(trunk_orientation_task);
    solver = nullptr;
  }
}

WalkTasks::~WalkTasks()
{
  remove_tasks();
}
};  // namespace placo