// Python binding for MoveIt's time-optimal path parameterization.
//
// MoveIt ships TOTG (Kunz & Stilman) as C++ only -- Humble has no moveit_py --
// while every runtime package here is ament_python. This module is the whole
// bridge: the Path and Trajectory classes take plain joint vectors and explicit
// per-joint limits, so no RobotModel, URDF or planning group is involved and the
// binding stays a pure function.
//
// It re-times a geometric path; it does not preserve the timing of its input.
// Modes that must keep taught timing use the spline path instead.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <Eigen/Core>

#include <cmath>
#include <list>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;

namespace
{
using trajectory_processing::Path;
using trajectory_processing::Trajectory;

Eigen::VectorXd to_eigen(const std::vector<double>& values)
{
  Eigen::VectorXd out(static_cast<Eigen::Index>(values.size()));
  for (std::size_t i = 0; i < values.size(); ++i)
    out[static_cast<Eigen::Index>(i)] = values[i];
  return out;
}

std::vector<double> from_eigen(const Eigen::VectorXd& values)
{
  return std::vector<double>(values.data(), values.data() + values.size());
}

/// Re-time `waypoints` subject to per-joint velocity and acceleration limits.
///
/// `max_deviation` is the blend tolerance at waypoints: 0.0 keeps the path
/// exactly through them, a positive value permits circular blends of at most
/// that radius and yields a smoother, faster traversal.
///
/// Returns the sampled trajectory plus the achieved duration. Sampling uses
/// TOTG's own analytic position/velocity/acceleration, so `resample_dt` costs
/// only output size -- it never changes the motion.
py::dict retime_path(const std::vector<std::vector<double>>& waypoints,
                     const std::vector<double>& max_velocity,
                     const std::vector<double>& max_acceleration,
                     double max_deviation,
                     double resample_dt,
                     double time_step)
{
  if (waypoints.size() < 2)
    throw std::invalid_argument("need at least two waypoints to re-time a path");
  if (max_velocity.empty())
    throw std::invalid_argument("max_velocity must not be empty");
  if (max_velocity.size() != max_acceleration.size())
    throw std::invalid_argument("max_velocity and max_acceleration must have the same length");

  const std::size_t joint_count = max_velocity.size();
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    if (waypoints[i].size() != joint_count)
      throw std::invalid_argument("waypoint " + std::to_string(i) + " has " +
                                  std::to_string(waypoints[i].size()) + " joints, expected " +
                                  std::to_string(joint_count));
  }
  for (std::size_t j = 0; j < joint_count; ++j)
  {
    if (!(max_velocity[j] > 0.0) || !std::isfinite(max_velocity[j]))
      throw std::invalid_argument("max_velocity[" + std::to_string(j) + "] must be finite and > 0");
    if (!(max_acceleration[j] > 0.0) || !std::isfinite(max_acceleration[j]))
      throw std::invalid_argument("max_acceleration[" + std::to_string(j) + "] must be finite and > 0");
  }
  if (!(resample_dt > 0.0))
    throw std::invalid_argument("resample_dt must be > 0");
  if (!(time_step > 0.0))
    throw std::invalid_argument("time_step must be > 0");
  if (max_deviation < 0.0)
    throw std::invalid_argument("max_deviation must be >= 0");

  std::list<Eigen::VectorXd> path_points;
  for (const auto& waypoint : waypoints)
    path_points.push_back(to_eigen(waypoint));

  const Path path(path_points, max_deviation);
  const Trajectory trajectory(path, to_eigen(max_velocity), to_eigen(max_acceleration), time_step);

  py::dict result;
  result["valid"] = trajectory.isValid();
  if (!trajectory.isValid())
  {
    // A TOTG failure is reported, never silently substituted: the caller decides
    // whether to fall back or refuse the motion.
    result["duration"] = 0.0;
    result["times"] = std::vector<double>{};
    result["positions"] = std::vector<std::vector<double>>{};
    result["velocities"] = std::vector<std::vector<double>>{};
    result["accelerations"] = std::vector<std::vector<double>>{};
    result["path_length"] = path.getLength();
    return result;
  }

  const double duration = trajectory.getDuration();
  std::vector<double> times;
  std::vector<std::vector<double>> positions;
  std::vector<std::vector<double>> velocities;
  std::vector<std::vector<double>> accelerations;

  for (double t = 0.0; t < duration; t += resample_dt)
  {
    times.push_back(t);
    positions.push_back(from_eigen(trajectory.getPosition(t)));
    velocities.push_back(from_eigen(trajectory.getVelocity(t)));
    accelerations.push_back(from_eigen(trajectory.getAcceleration(t)));
  }
  // Always land exactly on the endpoint; a truncated final step would leave the
  // arm short of the taught pose.
  times.push_back(duration);
  positions.push_back(from_eigen(trajectory.getPosition(duration)));
  velocities.push_back(from_eigen(trajectory.getVelocity(duration)));
  accelerations.push_back(from_eigen(trajectory.getAcceleration(duration)));

  result["duration"] = duration;
  result["times"] = times;
  result["positions"] = positions;
  result["velocities"] = velocities;
  result["accelerations"] = accelerations;
  result["path_length"] = path.getLength();
  return result;
}

}  // namespace

PYBIND11_MODULE(_totg, m)
{
  m.doc() = "Time-optimal path parameterization (MoveIt TOTG) for recorded joint paths.";
  m.def("retime_path", &retime_path, py::arg("waypoints"), py::arg("max_velocity"),
        py::arg("max_acceleration"), py::arg("max_deviation") = 0.0,
        py::arg("resample_dt") = 0.005, py::arg("time_step") = 0.001,
        "Re-time a joint-space path under per-joint velocity and acceleration limits.");
}
