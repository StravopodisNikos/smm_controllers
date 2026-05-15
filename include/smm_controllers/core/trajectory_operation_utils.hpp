#ifndef SMM_CONTROLLERS_TRAJECTORY_OPERATION_UTILS_HPP_
#define SMM_CONTROLLERS_TRAJECTORY_OPERATION_UTILS_HPP_

#include <string>

#include <Eigen/Dense>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace smm_controllers
{

enum class TrajectoryInterpolationMode
{
  SYNC_CUBIC,
  CUBIC_HERMITE
};

class TrajectoryOperationUtils
{
public:
  static TrajectoryInterpolationMode parseInterpolationMode(
    const std::string & mode);

  static std::string interpolationModeToString(
    TrajectoryInterpolationMode mode);

  static double pointTimeSec(
    const trajectory_msgs::msg::JointTrajectoryPoint & point);

  static bool sampleSegment(
    TrajectoryInterpolationMode mode,
    const trajectory_msgs::msg::JointTrajectoryPoint & p0,
    const trajectory_msgs::msg::JointTrajectoryPoint & p1,
    double t,
    std::size_t dof,
    Eigen::VectorXd & q,
    Eigen::VectorXd & qdot,
    Eigen::VectorXd & qddot);

  static bool sampleSynchronizedCubic(
    const trajectory_msgs::msg::JointTrajectoryPoint & p0,
    const trajectory_msgs::msg::JointTrajectoryPoint & p1,
    double t,
    std::size_t dof,
    Eigen::VectorXd & q,
    Eigen::VectorXd & qdot,
    Eigen::VectorXd & qddot);

  static bool sampleCubicHermite(
    const trajectory_msgs::msg::JointTrajectoryPoint & p0,
    const trajectory_msgs::msg::JointTrajectoryPoint & p1,
    double t,
    std::size_t dof,
    Eigen::VectorXd & q,
    Eigen::VectorXd & qdot,
    Eigen::VectorXd & qddot);
};

}  // namespace smm_controllers

#endif  // SMM_CONTROLLERS_TRAJECTORY_OPERATION_UTILS_HPP_