#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/loaned_command_interface.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace smm_controllers
{

std::vector<std::string> make_interface_names(
  const std::vector<std::string> & joint_names,
  const std::string & interface_name);

// ---------------------------------------------------------------------------
// Generic command-interface helper
// ---------------------------------------------------------------------------
bool write_vector_to_command_interfaces(
  std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces,
  const std::vector<std::string> & joint_names,
  const Eigen::VectorXd & command_vector,
  const rclcpp::Logger & logger,
  rclcpp::Clock & clock,
  const std::string & command_name);

// ---------------------------------------------------------------------------
// Generic JointState message helper
// ---------------------------------------------------------------------------
void fill_joint_state_msg(
  sensor_msgs::msg::JointState & msg,
  const std::vector<std::string> & joint_names,
  const Eigen::VectorXd & position,
  const Eigen::VectorXd & velocity,
  const Eigen::VectorXd & effort);

// ---------------------------------------------------------------------------
// Generic debug/error publishing helper
// ---------------------------------------------------------------------------
void fill_joint_error_state_msg(
  sensor_msgs::msg::JointState & msg,
  const std::vector<std::string> & joint_names,
  const Eigen::VectorXd & position_error,
  const Eigen::VectorXd & velocity_error,
  const Eigen::VectorXd & effort_command);

void publish_scalar_error_topics(
  const std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> & pubs,
  std::vector<std_msgs::msg::Float64> & msgs,
  const Eigen::VectorXd & values);

}  // namespace smm_controllers