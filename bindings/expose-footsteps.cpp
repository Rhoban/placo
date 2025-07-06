#include <pinocchio/fwd.hpp>

#include "expose-utils.hpp"
#include "module.h"
#include "doxystub.h"
#include "placo/humanoid/footsteps_planner.h"
#include "placo/humanoid/footsteps_planner_naive.h"
#include "placo/humanoid/footsteps_planner_repetitive.h"
#include <Eigen/Dense>
#include <boost/python.hpp>
#include <eigenpy/eigen-to-python.hpp>

using namespace boost::python;
using namespace placo::humanoid;
using namespace placo;

void exposeFootsteps()
{
  enum_<HumanoidRobot::Side>("HumanoidRobot_Side")
      .value("left", HumanoidRobot::Side::Left)
      .value("right", HumanoidRobot::Side::Right);

  class__<FootstepsPlanner::Footstep>("Footstep", init<double, double>())
      .add_property("side", &FootstepsPlanner::Footstep::side, &FootstepsPlanner::Footstep::side)
      .add_property("foot_length", &FootstepsPlanner::Footstep::foot_length, &FootstepsPlanner::Footstep::foot_length)
      .add_property("foot_width", &FootstepsPlanner::Footstep::foot_width, &FootstepsPlanner::Footstep::foot_width)
      .add_property(
          "frame", +[](FootstepsPlanner::Footstep& footstep) { return footstep.frame; },
          &FootstepsPlanner::Footstep::frame)
      .def(
          "set_frame_xy",
          +[](FootstepsPlanner::Footstep& footstep, double x, double y) {
            footstep.frame.translation().x() = x;
            footstep.frame.translation().y() = y;
          })
      .def("support_polygon", &FootstepsPlanner::Footstep::support_polygon)
      .def("overlap", &FootstepsPlanner::Footstep::overlap)
      .def("polygon_contains", &FootstepsPlanner::Footstep::polygon_contains)
      .staticmethod("polygon_contains");

  class__<FootstepsPlanner::Support>("Support", init<>())
      .def("support_polygon", &FootstepsPlanner::Support::support_polygon)
      .def("frame", &FootstepsPlanner::Support::frame)
      .def("footstep_frame", &FootstepsPlanner::Support::footstep_frame)
      .def("apply_offset", &FootstepsPlanner::Support::apply_offset)
      .def("side", &FootstepsPlanner::Support::side)
      .def("is_both", &FootstepsPlanner::Support::is_both)
      .def(
          "set_start", +[](FootstepsPlanner::Support& support, bool b) { support.start = b; })
      .def(
          "set_end", +[](FootstepsPlanner::Support& support, bool b) { support.end = b; })
      .add_property("footsteps", &FootstepsPlanner::Support::footsteps, &FootstepsPlanner::Support::footsteps)
      .add_property("t_start", &FootstepsPlanner::Support::t_start, &FootstepsPlanner::Support::t_start)
      .add_property("elapsed_ratio", &FootstepsPlanner::Support::elapsed_ratio,
                    &FootstepsPlanner::Support::elapsed_ratio)
      .add_property("time_ratio", &FootstepsPlanner::Support::time_ratio, &FootstepsPlanner::Support::time_ratio)
      .add_property("start", &FootstepsPlanner::Support::start, &FootstepsPlanner::Support::start)
      .add_property("end", &FootstepsPlanner::Support::end, &FootstepsPlanner::Support::end)
      .add_property("target_world_dcm", &FootstepsPlanner::Support::target_world_dcm,
                    &FootstepsPlanner::Support::target_world_dcm);

  class__<FootstepsPlanner, boost::noncopyable>("FootstepsPlanner", no_init)
      .def("make_supports", &FootstepsPlanner::make_supports)
      .def("opposite_footstep", &FootstepsPlanner::opposite_footstep)
      .def("clipped_opposite_footstep", &FootstepsPlanner::clipped_opposite_footstep)
      .def(
          "truncate_supports", +[](const std::vector<FootstepsPlanner::Support>& supports, int index, bool add_end) {
            if (index > (int)supports.size())
            {
              throw std::out_of_range("Index out of range for supports");
            }

            std::vector<FootstepsPlanner::Support> head;
            for (int i = 0; i < index; ++i)
            {
              head.push_back(supports[i]);
            }

            if (add_end && !head.back().end)
            {
              FootstepsPlanner::Support end_support({ head.back().footsteps[0] });
              end_support.end = true;
              if (end_support.footsteps.size() == 1)
              {
                if (HumanoidRobot::other_side(supports[index].footsteps[0].side) == end_support.footsteps[0].side)
                {
                  end_support.footsteps.push_back(supports[index].footsteps[0]);
                }
                else
                {
                  end_support.footsteps.push_back(supports[index].footsteps[1]);
                }
              }
              head.push_back(end_support);
            }
            return head;
          });

  class__<FootstepsPlannerNaive, bases<FootstepsPlanner>>("FootstepsPlannerNaive", init<HumanoidParameters&>())
      .def("plan", &FootstepsPlannerNaive::plan)
      .def("configure", &FootstepsPlannerNaive::configure);

  class__<FootstepsPlannerRepetitive, bases<FootstepsPlanner>>("FootstepsPlannerRepetitive",
                                                               init<HumanoidParameters&>())
      .def("plan", &FootstepsPlannerRepetitive::plan)
      .def("configure", &FootstepsPlannerRepetitive::configure);

  // Exposing vector of footsteps
  exposeStdVector<FootstepsPlanner::Footstep>("Footsteps");
  exposeStdVector<FootstepsPlanner::Support>("Supports");
}
