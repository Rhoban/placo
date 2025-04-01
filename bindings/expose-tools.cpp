#include <pinocchio/fwd.hpp>

#include <Eigen/Dense>
#include <boost/python.hpp>
#include "module.h"
#include "doxystub/registry.h"
#include "placo/tools/utils.h"
#include "placo/tools/cubic_spline.h"
#include "placo/tools/cubic_spline_3d.h"
#include "placo/tools/axises_mask.h"
#include "placo/tools/prioritized.h"
#include "placo/tools/directions.h"
#include "placo/tools/polynom.h"
#include "placo/tools/segment.h"
#include "expose-utils.hpp"
#ifdef HAVE_RHOBAN_UTILS
#include "rhoban_utils/history/history.h"
#endif
#include <eigenpy/eigen-to-python.hpp>

using namespace boost::python;
using namespace placo::tools;

#ifdef HAVE_RHOBAN_UTILS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(loadReplays_overloads, loadReplays, 1, 2);
#endif

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(set_axises_overloads, set_axises, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(configure_overloads, configure, 2, 3);
BOOST_PYTHON_FUNCTION_OVERLOADS(directions_3d_overloads, directions_3d, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(value_overloads, value, 1, 2);

void exposeTools()
{
  def("interpolate_frames", &interpolate_frames, args("frameA", "frameB", "AtoB"),
      "Interpolate between `frameA` and `frameB`. `AtoB` is a scalar from 0 (frame A) to 1 (frame B)");
  def("wrap_angle", &wrap_angle, args("angle"), "Wrap given <angle> (in radians) between -PI and PI");
  def("rotation_from_axis", &rotation_from_axis, args("axis", "vector"),
      "Make a rotation matrix such that `axis` (x, y or z) is oriented towards `vector`");
  def("frame_yaw", &frame_yaw, args("rotation_matrix"), "Computes the yaw heading for given `rotation_matrix`");
  def("flatten_on_floor", &flatten_on_floor, args("transformation"),
      "Put the given `transformation` on floor, with no pitch and roll");
  def("optimal_transformation", &optimal_transformation, args("points_in_A", "points_in_B"),
      "Provided two sets of points in frame A and B, finds the optimal transformation explaining those positions");
  def("directions_2d", &directions_2d, args("n"), "Generate `n` uniformly distributed 2D vectors");
  def("directions_3d", &directions_3d,
      directions_3d_overloads(args("n"), "Generate `n` uniformly distributed 3D vectors"));

  exposeStdVector<int>("vector_int");
  exposeStdVector<double>("vector_double");
  exposeStdVector<std::string>("vector_string");
  exposeStdVector<Eigen::MatrixXd>("vector_MatrixXd");

  // exposeStdMap<std::string, double>("map_string_double");

  class_<std::map<std::string, double>>("map_string_double").def(map_indexing_suite<std::map<std::string, double>>());

  class__<AxisesMask>("AxisesMask", init<>())
      .def<void (AxisesMask::*)(std::string, std::string)>  //
      ("set_axises", &AxisesMask::set_axises,
       set_axises_overloads(args("self", "axises", "frame"), "Specify the `axises` to mask (keep), in `frame` (task, "
                                                             "local or custom)"))
      .def_readwrite("R_local_world", &AxisesMask::R_local_world,
                     np_docstring("Transformation from world to local frame"))
      .def_readwrite("R_custom_world", &AxisesMask::R_custom_world,
                     np_docstring("Transformation from world to custom frame"))
      .def("apply", &AxisesMask::apply, args("self", "M"), "Apply the masking to M, keeping specified axises");

  class__<Prioritized, boost::noncopyable>("Prioritized", no_init)
      .add_property("name", &Prioritized::name)
      .add_property(
          "priority", +[](Prioritized& pri) { return pri.priority_name(); })
      .def<void (Prioritized::*)(std::string, std::string, double)>("configure", &Prioritized::configure,
                                                                    configure_overloads());

  class__<CubicSpline>("CubicSpline", init<optional<bool>>())
      .def("pos", &CubicSpline::pos)
      .def("vel", &CubicSpline::vel)
      .def("acc", &CubicSpline::acc)
      .def("add_point", &CubicSpline::add_point)
      .def("clear", &CubicSpline::clear)
      .def("duration", &CubicSpline::duration);

  class__<CubicSpline3D>("CubicSpline3D")
      .def("pos", &CubicSpline3D::pos)
      .def("vel", &CubicSpline3D::vel)
      .def("acc", &CubicSpline3D::acc)
      .def("add_point", &CubicSpline3D::add_point, boost::python::args("self", "t", "x", "dx"),
           "Adds a point un the cubic spline")
      .def("clear", &CubicSpline3D::clear)
      .def("duration", &CubicSpline3D::duration);

  class__<Polynom>("Polynom", init<Eigen::VectorXd>())
      .def("value", &Polynom::value, value_overloads())
      .def("derivative_coefficient", &Polynom::derivative_coefficient)
      .staticmethod("derivative_coefficient")
      .def_readwrite("coefficients", &Polynom::coefficients);

  class__<Segment>("Segment", init<Eigen::Vector2d, Eigen::Vector2d>())
      .add_property("start", &Segment::start, &Segment::start)
      .add_property("end", &Segment::end, &Segment::start)
      .def(
          "is_parallel", +[](Segment& s1, const Segment& s2) { return s1.is_parallel(s2); })
      .def(
          "is_point_aligned", +[](Segment& s, const Eigen::Vector2d& point) { return s.is_point_aligned(point); })
      .def(
          "is_collinear", +[](Segment& s1, const Segment& s2) { return s1.is_collinear(s2); })
      .def(
          "is_point_in_segment", +[](Segment& s, const Eigen::Vector2d& point) { return s.is_point_in_segment(point); })
      .def("intersects", &Segment::intersects)
      .def("line_pass_through", &Segment::line_pass_through)
      .def("half_line_pass_through", &Segment::half_line_pass_through)
      .def("lines_intersection", &Segment::lines_intersection);

#ifdef HAVE_RHOBAN_UTILS
  using namespace rhoban_utils;

  // History collection
  class__<HistoryCollection>("HistoryCollection")
      .def("loadReplays", &HistoryCollection::loadReplays, loadReplays_overloads())
      .def("exportToCSV", &HistoryCollection::exportToCSV)
      .def("smallestTimestamp", &HistoryCollection::smallestTimestamp)
      .def("biggestTimestamp", &HistoryCollection::biggestTimestamp)
      .def("startNamedLog", &HistoryCollection::startNamedLog)
      .def("stopNamedLog", &HistoryCollection::stopNamedLog)
      .def("getTimestamps", &HistoryCollection::getTimestamps)
      .def(
          "number", +[](HistoryCollection& collection, std::string name,
                        double t) { return collection.number(name)->interpolate(t); })
      .def(
          "angle", +[](HistoryCollection& collection, std::string name,
                       double t) { return collection.angle(name)->interpolate(t); })
      .def(
          "pose", +[](HistoryCollection& collection, std::string name,
                      double t) { return collection.pose(name)->interpolate(t); })
      .def(
          "bool", +[](HistoryCollection& collection, std::string name,
                      double t) { return collection.boolean(name)->interpolate(t); })
      .def(
          "push_number", +[](HistoryCollection& collection, std::string name, double t,
                             double value) { collection.number(name)->pushValue(t, value); })
      .def(
          "push_angle", +[](HistoryCollection& collection, std::string name, double t,
                            double value) { collection.angle(name)->pushValue(t, value); })
      .def(
          "push_pose", +[](HistoryCollection& collection, std::string name, double t,
                           Eigen::Affine3d value) { collection.pose(name)->pushValue(t, value); })
      .def(
          "push_bool", +[](HistoryCollection& collection, std::string name, double t,
                           bool value) { collection.boolean(name)->pushValue(t, value); })
      .def(
          "get_sequence",
          +[](HistoryCollection& collection, std::vector<std::string> entries, double start_t, double dt, int length) {
            Eigen::MatrixXd result(length, entries.size());

            for (unsigned int i = 0; i < length; i++)
            {
              double t = start_t + i * dt;
              for (unsigned int j = 0; j < entries.size(); j++)
              {
                result(i, j) = collection.number(entries[j])->interpolate(t);
              }
            }

            return result;
          })

      ;
#endif
}
