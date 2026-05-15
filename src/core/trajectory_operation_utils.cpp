#include "smm_controllers/core/trajectory_operation_utils.hpp"

#include <algorithm>
#include <stdexcept>

namespace smm_controllers
{

TrajectoryInterpolationMode TrajectoryOperationUtils::parseInterpolationMode(
  const std::string & mode)
{
  if (mode == "sync_cubic" || mode == "synchronized_cubic") {
    return TrajectoryInterpolationMode::SYNC_CUBIC;
  }

  if (mode == "cubic_hermite" || mode == "hermite") {
    return TrajectoryInterpolationMode::CUBIC_HERMITE;
  }

  throw std::runtime_error(
    "Unsupported trajectory interpolation mode: " + mode +
    ". Valid modes: sync_cubic, cubic_hermite.");
}

std::string TrajectoryOperationUtils::interpolationModeToString(
  TrajectoryInterpolationMode mode)
{
  switch (mode) {
    case TrajectoryInterpolationMode::SYNC_CUBIC:
      return "sync_cubic";

    case TrajectoryInterpolationMode::CUBIC_HERMITE:
      return "cubic_hermite";
  }

  return "unknown";
}

double TrajectoryOperationUtils::pointTimeSec(
  const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

bool TrajectoryOperationUtils::sampleSegment(
  TrajectoryInterpolationMode mode,
  const trajectory_msgs::msg::JointTrajectoryPoint & p0,
  const trajectory_msgs::msg::JointTrajectoryPoint & p1,
  double t,
  std::size_t dof,
  Eigen::VectorXd & q,
  Eigen::VectorXd & qdot,
  Eigen::VectorXd & qddot)
{
  switch (mode) {
    case TrajectoryInterpolationMode::SYNC_CUBIC:
      return sampleSynchronizedCubic(p0, p1, t, dof, q, qdot, qddot);

    case TrajectoryInterpolationMode::CUBIC_HERMITE:
      return sampleCubicHermite(p0, p1, t, dof, q, qdot, qddot);
  }

  return false;
}

bool TrajectoryOperationUtils::sampleSynchronizedCubic(
  const trajectory_msgs::msg::JointTrajectoryPoint & p0,
  const trajectory_msgs::msg::JointTrajectoryPoint & p1,
  double t,
  std::size_t dof,
  Eigen::VectorXd & q,
  Eigen::VectorXd & qdot,
  Eigen::VectorXd & qddot)
{
  const double t0 = pointTimeSec(p0);
  const double t1 = pointTimeSec(p1);
  const double T = t1 - t0;

  if (T <= 0.0) {
    return false;
  }

  double tau = (t - t0) / T;
  tau = std::clamp(tau, 0.0, 1.0);

  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;

  const double s = 3.0 * tau2 - 2.0 * tau3;
  const double sdot = (6.0 * tau - 6.0 * tau2) / T;
  const double sddot = (6.0 - 12.0 * tau) / (T * T);

  q.resize(static_cast<Eigen::Index>(dof));
  qdot.resize(static_cast<Eigen::Index>(dof));
  qddot.resize(static_cast<Eigen::Index>(dof));

  for (std::size_t i = 0; i < dof; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    const double q0 = p0.positions[i];
    const double q1 = p1.positions[i];
    const double delta_q = q1 - q0;

    q(idx) = q0 + s * delta_q;
    qdot(idx) = sdot * delta_q;
    qddot(idx) = sddot * delta_q;
  }

  return true;
}

bool TrajectoryOperationUtils::sampleCubicHermite(
  const trajectory_msgs::msg::JointTrajectoryPoint & p0,
  const trajectory_msgs::msg::JointTrajectoryPoint & p1,
  double t,
  std::size_t dof,
  Eigen::VectorXd & q,
  Eigen::VectorXd & qdot,
  Eigen::VectorXd & qddot)
{
  const double t0 = pointTimeSec(p0);
  const double t1 = pointTimeSec(p1);
  const double T = t1 - t0;

  if (T <= 0.0) {
    return false;
  }

  double s = (t - t0) / T;
  s = std::clamp(s, 0.0, 1.0);

  const double s2 = s * s;
  const double s3 = s2 * s;

  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;

  const double dh00 = 6.0 * s2 - 6.0 * s;
  const double dh10 = 3.0 * s2 - 4.0 * s + 1.0;
  const double dh01 = -6.0 * s2 + 6.0 * s;
  const double dh11 = 3.0 * s2 - 2.0 * s;

  const double ddh00 = 12.0 * s - 6.0;
  const double ddh10 = 6.0 * s - 4.0;
  const double ddh01 = -12.0 * s + 6.0;
  const double ddh11 = 6.0 * s - 2.0;

  q.resize(static_cast<Eigen::Index>(dof));
  qdot.resize(static_cast<Eigen::Index>(dof));
  qddot.resize(static_cast<Eigen::Index>(dof));

  for (std::size_t i = 0; i < dof; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);

    const double q0 = p0.positions[i];
    const double q1 = p1.positions[i];

    const double v0 =
      (!p0.velocities.empty()) ? p0.velocities[i] : 0.0;

    const double v1 =
      (!p1.velocities.empty()) ? p1.velocities[i] : 0.0;

    q(idx) =
      h00 * q0 +
      h10 * T * v0 +
      h01 * q1 +
      h11 * T * v1;

    qdot(idx) =
      (dh00 * q0 +
       dh10 * T * v0 +
       dh01 * q1 +
       dh11 * T * v1) / T;

    qddot(idx) =
      (ddh00 * q0 +
       ddh10 * T * v0 +
       ddh01 * q1 +
       ddh11 * T * v1) / (T * T);
  }

  return true;
}

}  // namespace smm_controllers